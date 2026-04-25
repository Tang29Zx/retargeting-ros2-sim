import os
import cv2
import numpy as np
import torch
from hybrik.utils.config import update_config
from torchvision.models.detection import fasterrcnn_resnet50_fpn
from easydict import EasyDict as edict
from hybrik.utils.presets import SimpleTransform3DSMPLCam
from hybrik.models import builder
from hybrik.utils.vis import get_max_iou_box, get_one_box
from torchvision import transforms as T

class HybrikInferencer:
    def __init__(self, root_path, device = None):
        if device is None:
            if torch.cuda.is_available():
                self.device = torch.device("cuda:0")
            else:
                self.device = torch.device("cpu")
        else:
            self.device = torch.device(device)
        self.ROOT_DIR = root_path
        self.ckpt = self._get_ckpt_init()
        self.cfg = self._get_cfg_init()
        self.transformation = self._model_preprocess_init()
        self.hybrik_model = self._hybrik_build_init()
        self.prev_box = None
        self.det_model = self._det_preprocess_init()

        self.det_transform = T.Compose([T.ToTensor()])
        self.hybrik_model.to(self.device)
        self.hybrik_model.eval()
    
    def _get_cfg_init(self):
        path = os.path.join(
            self.ROOT_DIR, 'repos', 'perception', 
            'Hybrik', 'configs', '256x192_adam_lr1e-3-hrw48_cam_2x_w_pw3d_3dhp.yaml')
        cfg = update_config(path)
        return cfg
    
    def _get_ckpt_init(self):
        path = os.path.join(
            self.ROOT_DIR, 'repos', 'perception', 
            'Hybrik', 'pretrained_models', 'hybrik_hrnet.pth')
        return path
    
    def _model_preprocess_init(self):
        bbox_3d_shape = getattr(self.cfg.MODEL, 'BBOX_3D_SHAPE', (2000, 2000, 2000))
        bbox_3d_shape = [item * 1e-3 for item in bbox_3d_shape]
        dummy_set = edict({
            'joint_pairs_17': None,
            'joint_pairs_24': None,
            'joint_pairs_29': None,
            'bbox_3d_shape': bbox_3d_shape
            })

        transformation = SimpleTransform3DSMPLCam(
            dummy_set, scale_factor=self.cfg.DATASET.SCALE_FACTOR,
            color_factor=self.cfg.DATASET.COLOR_FACTOR,
            occlusion=self.cfg.DATASET.OCCLUSION,
            input_size=self.cfg.MODEL.IMAGE_SIZE,
            output_size=self.cfg.MODEL.HEATMAP_SIZE,
            depth_dim=self.cfg.MODEL.EXTRA.DEPTH_DIM,
            bbox_3d_shape=bbox_3d_shape,
            rot=self.cfg.DATASET.ROT_FACTOR, sigma=self.cfg.MODEL.EXTRA.SIGMA,
            train=False, add_dpg=False,
            loss_type=self.cfg.LOSS['TYPE'])
        
        return transformation

    def _hybrik_build_init(self):
        hybrik_model = builder.build_sppe(self.cfg.MODEL)
        
        save_dict = torch.load(self.ckpt, map_location='cpu')
        if type(save_dict) == dict:
            model_dict = save_dict['model']
            hybrik_model.load_state_dict(model_dict)
        else:
            hybrik_model.load_state_dict(save_dict)
        
        return hybrik_model

    def _det_preprocess_init(self):
        det_model = fasterrcnn_resnet50_fpn(pretrained=True)
        det_model.to(self.device)
        det_model.eval()
        return det_model

    @torch.no_grad()
    def _detection(self, input_image):
        det_input = self.det_transform(input_image).to(self.device)
        det_output = self.det_model([det_input])[0]

        if self.prev_box is None:
            tight_bbox = get_one_box(det_output)
            if tight_bbox is None:
                return None
        else:
            tight_bbox = get_max_iou_box(det_output, self.prev_box)
            if tight_bbox is None:
                self.prev_box = None
                return None
        self.prev_box = tight_bbox

        return tight_bbox

    @torch.no_grad()
    def run_model(self, input_image_BGR):
        input_image = cv2.cvtColor(input_image_BGR, cv2.COLOR_BGR2RGB)
        tight_bbox = self._detection(input_image)
        if tight_bbox is None:
            return None
        pose_input, bbox, img_center = self.transformation.test_transform(
            input_image, tight_bbox)
        pose_input = pose_input.to(self.device)[None, :, :, :]
        pose_output = self.hybrik_model(
            pose_input, flip_test=True,
            bboxes=torch.from_numpy(np.array(bbox)).to(pose_input.device).unsqueeze(0).float(),
            img_center=torch.from_numpy(img_center).to(pose_input.device).unsqueeze(0).float()
            )

        return pose_output
        