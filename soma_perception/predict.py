"""
Copyright (C) 2019 NVIDIA Corporation.  All rights reserved.
Licensed under the CC BY-NC-SA 4.0 license (https://creativecommons.org/licenses/by-nc-sa/4.0/legalcode).
"""

from __future__ import absolute_import
from __future__ import division
import argparse
from functools import partial
from config import cfg, assert_and_infer_cfg
import logging
import math
import os
import sys

import torch
import numpy as np
from datasets import cityscapes, rellis
import yaml
from utils.misc import AverageMeter, prep_experiment, evaluate_eval, fast_hist
from utils.f_boundary import eval_mask_boundary
import datasets
import loss
import network
import optimizer
from tqdm import tqdm
from PIL import Image
import transforms.joint_transforms as joint_transforms
import transforms.transforms as extended_transforms
import torchvision.transforms as standard_transforms

# Argument Parser
parser = argparse.ArgumentParser(description='GSCNN')
parser.add_argument('--lr', type=float, default=0.01)
parser.add_argument('--arch', type=str, default='network.gscnn.GSCNN')
parser.add_argument('--dataset', type=str, default='rellis')
parser.add_argument('--cv', type=int, default=0,
                    help='cross validation split')
parser.add_argument('--joint_edgeseg_loss', action='store_true', default=True,
                    help='joint loss')
parser.add_argument('--img_wt_loss', action='store_true', default=False,
                    help='per-image class-weighted loss')
parser.add_argument('--batch_weighting', action='store_true', default=False,
                    help='Batch weighting for class')
parser.add_argument('--eval_thresholds', type=str, default='0.0005,0.001875,0.00375,0.005',
                    help='Thresholds for boundary evaluation')
parser.add_argument('--rescale', type=float, default=1.0,
                    help='Rescaled LR Rate')
parser.add_argument('--repoly', type=float, default=1.5,
                    help='Rescaled Poly')

parser.add_argument('--edge_weight', type=float, default=1.0,
                    help='Edge loss weight for joint loss')
parser.add_argument('--seg_weight', type=float, default=1.0,
                    help='Segmentation loss weight for joint loss')
parser.add_argument('--att_weight', type=float, default=1.0,
                    help='Attention loss weight for joint loss')
parser.add_argument('--dual_weight', type=float, default=1.0,
                    help='Dual loss weight for joint loss')

parser.add_argument('--evaluate', action='store_true', default=False)

parser.add_argument("--local_rank", default=0, type=int)

parser.add_argument('--sgd', action='store_true', default=True)
parser.add_argument('--sgd_finetuned',action='store_true',default=False)
parser.add_argument('--adam', action='store_true', default=False)
parser.add_argument('--amsgrad', action='store_true', default=False)

parser.add_argument('--trunk', type=str, default='resnet101',
                    help='trunk model, can be: resnet101 (default), resnet50')
parser.add_argument('--max_epoch', type=int, default=175)
parser.add_argument('--start_epoch', type=int, default=0)
parser.add_argument('--color_aug', type=float,
                    default=0.25, help='level of color augmentation')
parser.add_argument('--rotate', type=float,
                    default=0, help='rotation')
parser.add_argument('--gblur', action='store_true', default=True)
parser.add_argument('--bblur', action='store_true', default=False) 
parser.add_argument('--lr_schedule', type=str, default='poly',
                    help='name of lr schedule: poly')
parser.add_argument('--poly_exp', type=float, default=1.0,
                    help='polynomial LR exponent')
parser.add_argument('--bs_mult', type=int, default=1)
parser.add_argument('--bs_mult_val', type=int, default=2)
parser.add_argument('--crop_size', type=int, default=720,
                    help='training crop size')
parser.add_argument('--pre_size', type=int, default=None,
                    help='resize image shorter edge to this before augmentation')
parser.add_argument('--scale_min', type=float, default=0.5,
                    help='dynamically scale training images down to this size')
parser.add_argument('--scale_max', type=float, default=2.0,
                    help='dynamically scale training images up to this size')
parser.add_argument('--weight_decay', type=float, default=1e-4)
parser.add_argument('--momentum', type=float, default=0.9)
parser.add_argument('--snapshot', type=str, default=None)
parser.add_argument('--restore_optimizer', action='store_true', default=False)
parser.add_argument('--exp', type=str, default='default',
                    help='experiment directory name')
parser.add_argument('--tb_tag', type=str, default='',
                    help='add tag to tb dir')
parser.add_argument('--ckpt', type=str, default='logs/ckpt')
parser.add_argument('--tb_path', type=str, default='logs/tb')
parser.add_argument('--syncbn', action='store_true', default=True,
                    help='Synchronized BN')
parser.add_argument('--dump_augmentation_images', action='store_true', default=False,
                    help='Synchronized BN')
parser.add_argument('--test_mode', action='store_true', default=False,
                    help='minimum testing (1 epoch run ) to verify nothing failed')
parser.add_argument('--mode',type=str,default="test")                    
parser.add_argument('--test_sv_path', type=str, default="./prediction")
parser.add_argument('--checkpoint_path',type=str,default="./checkpoints/gscnn_best.pth")
parser.add_argument('-wb', '--wt_bound', type=float, default=1.0)
parser.add_argument('--maxSkip', type=int, default=0)
parser.add_argument('--data-cfg', help='data config (kitti format)',
                    default='/home/hayashi/zwy/RELLIS-3D/benchmarks/GSCNN-master/rellis.yaml',
                    type=str)
parser.add_argument('--viz', dest='viz',
                    help="Save color predictions to disk",
                    action='store_true')
args = parser.parse_args()
args.best_record = {'epoch': -1, 'iter': 0, 'val_loss': 1e10, 'acc': 0,
                        'acc_cls': 0, 'mean_iu': 0, 'fwavacc': 0}


def convert_label(label, inverse=False):
    label_mapping = {0: 0,
                     1: 0,
                     3: 1,
                     4: 2,
                     5: 3,
                     6: 4,
                     7: 5,
                     8: 6,
                     9: 7,
                     10: 8,
                     12: 9,
                     15: 10,
                     17: 11,
                     18: 12,
                     19: 13,
                     23: 14,
                     27: 15,
                    #  29: 1,
                    #  30: 1,
                     31: 16,
                    #  32: 4,
                     33: 17,
                     34: 18}
    temp = label.copy()
    if inverse:
        for v,k in label_mapping.items():
            temp[label == k] = v
    else:
        for k, v in label_mapping.items():
            temp[label == k] = v
    return temp

def convert_color(label, color_map):
        temp = np.zeros(label.shape + (3,)).astype(np.uint8)
        for k,v in color_map.items():
            temp[label == k] = v
        return temp

#Enable CUDNN Benchmarking optimization
torch.backends.cudnn.benchmark = True
args.world_size = 1
#Test Mode run two epochs with a few iterations of training and val
if args.test_mode:
    args.max_epoch = 2

if 'WORLD_SIZE' in os.environ:
    args.world_size = int(os.environ['WORLD_SIZE'])
    print("Total world size: ", int(os.environ['WORLD_SIZE']))

def main():
    '''
    Main Function

    '''

    #Set up the Arguments, Tensorboard Writer, Dataloader, Loss Fn, Optimizer
    assert_and_infer_cfg(args)
    # writer = prep_experiment(args,parser)
    # train_loader, val_loader, train_obj = datasets.setup_loaders(args)
    args.dataset_cls  = rellis
    criterion, criterion_val = loss.get_loss(args)
    net = network.get_net(args, criterion)
    # optim, scheduler = optimizer.get_optimizer(args, net)

    torch.cuda.empty_cache()
    test_sv_path = args.test_sv_path
    # print(f"Saving prediction {test_sv_path}")
    net.eval()

    try:
        print("Opening config file %s" % args.data_cfg)
        CFG = yaml.safe_load(open(args.data_cfg, 'r'))
    except Exception as e:
        print(e)
        print("Error opening yaml file.")
        quit()
    id_color_map = CFG["color_map"]
    '--------------load color map using yaml file'


    # for vi, data in enumerate(tqdm(val_loader)):
    #     input, mask, img_name, img_path = data
    #     assert len(input.size()) == 4 and len(mask.size()) == 3
    #     assert input.size()[2:] == mask.size()[1:]
    #     b, h, w = mask.size()

    #     batch_pixel_size = input.size(0) * input.size(2) * input.size(3)
    #     input, mask_cuda = input.cuda(), mask.cuda()
    # print(input)





    input = Image.open('./test.jpg').convert('RGB')
    mean_std = ([0.496588, 0.59493099, 0.53358843], [0.496588, 0.59493099, 0.53358843])
    target_transform = extended_transforms.MaskToTensor()
    val_input_transform = standard_transforms.Compose([
    standard_transforms.ToTensor(),
    standard_transforms.Normalize(*mean_std)
])
    input = val_input_transform(input).unsqueeze(0).cuda()
    # input = transforms.ToTensor()(input)
    # print(input)


    with torch.no_grad():
        seg_out, edge_out = net(input)    # output = (1, 19, 713, 713)

    seg_predictions = seg_out.data.cpu().numpy()
    edge_predictions = edge_out.cpu().numpy()

    img_path = ['test.jpg',]
    i = 0
    _,file_name = os.path.split(img_path[i])
    file_name = file_name.replace("jpg","png")
    seq = img_path[i][:4]
    seg_path = os.path.join(test_sv_path,"labels")
    if not os.path.exists(seg_path): 
        os.makedirs(seg_path)

    seg_arg = np.argmax(seg_predictions[i],axis=0).astype(np.uint8)
    seg_arg = convert_label(seg_arg,True)

    seg_img = np.stack((seg_arg,seg_arg,seg_arg),axis=2)
    seg_img = Image.fromarray(seg_img)
    print(seg_path)
    seg_img.save(os.path.join(seg_path,file_name))

    args.viz = 1



    if args.viz:
        edge_arg = np.argmax(edge_predictions[i],axis=0).astype(np.uint8)
        edge_img = np.stack((edge_arg,edge_arg,edge_arg),axis=2)

        edge_path = os.path.join(test_sv_path,"edge",seq)
        #edgenp_path = os.path.join(test_sv_path,"gscnn","edgenp",seq)
        if not os.path.exists(edge_path):
            os.makedirs(edge_path)
            #os.makedirs(edgenp_path)
        edge_img = Image.fromarray(edge_img)
        edge_img.save(os.path.join(edge_path,file_name))                           
        print(edge_path)
        color_label = convert_color(seg_arg,id_color_map)
        color_path = os.path.join(test_sv_path,"color",seq)
        if not os.path.exists(color_path):
            os.makedirs(color_path)
        color_label = convert_color(seg_arg,id_color_map)
        color_label = Image.fromarray(color_label)
        print(color_path)
        color_label.save(os.path.join(color_path,file_name))                    


if __name__ == '__main__':
    main()