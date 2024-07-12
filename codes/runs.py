import os
import json
import logging
import argparse
import pickle
import numpy as np
import torch
import xml.etree.ElementTree as ET
import re
from torch.utils.data import DataLoader
from utils.tester import ModelTester

from datasets.GraspNet import *

from models import KGEModel, ModE, HAKE
# from models_grasp.architectures import KPCNN_G, KPFCNN
from models_grasp.architectures_pre_yf import KPCNN_G, KPFCNN

from data import TrainDataset, BatchType, ModeType, DataReader
from data import BidirectionalOneShotIterator
from config import Config

def parse_args(args=None):
    parser = argparse.ArgumentParser(
        description='Training and Testing Knowledge Graph Embedding Models',
        usage='runs.py [<args>] [-h | --help]'
    )

    parser.add_argument('--do_train', action='store_true')
    parser.add_argument('--do_valid', action='store_true')
    # parser.add_argument('--do_test', action='store_true')
    parser.add_argument('--do_test', action='store', nargs='?', const=True, default=True, type=bool)  #yf

    # parser.add_argument('--data_path', type=str, default="/home/yf/code/KGE-HAKE-master/data/FB15k-237/")
    parser.add_argument('--data_path', type=str, default='/home/yf/code/KGE-HAKE-master/F2F-V2-V1')
    parser.add_argument('--model', default='HAKE', type=str)

    parser.add_argument('-n', '--negative_sample_size', default=50, type=int)
    # parser.add_argument('-n', '--negative_sample_size', default=3, type=int)
    parser.add_argument('-d', '--hidden_dim', default=500, type=int)
    parser.add_argument('-g', '--gamma', default=12.0, type=float)
    parser.add_argument('-a', '--adversarial_temperature', default=1.0, type=float)
    parser.add_argument('-b', '--batch_size', default=1024, type=int)
    parser.add_argument('--test_batch_size', default=4, type=int, help='valid/test batch size')
    parser.add_argument('-mw', '--modulus_weight', default=1, type=float)
    parser.add_argument('-pw', '--phase_weight', default=0.5, type=float)

    parser.add_argument('-lr', '--learning_rate', default=0.0001, type=float)
    parser.add_argument('-cpu', '--cpu_num', default=10, type=int)
    # parser.add_argument('-init', '--init_checkpoint', default='/home/yf/code/KGE-HAKE-master/HAKE_F2F-V2', type=str)
    parser.add_argument('-init', '--init_checkpoint', default=False, type=str)
    parser.add_argument('-save', '--save_path', default="/home/yf/code/KGE-HAKE-master/models/HAKE_F2F-V2/", type=str)
    parser.add_argument('--max_steps', default=2000, type=int)

    parser.add_argument('--save_checkpoint_steps', default=4000, type=int)
    parser.add_argument('--valid_steps', default=1000, type=int)
    parser.add_argument('--log_steps', default=100, type=int, help='train log every xx steps')
    parser.add_argument('--test_log_steps', default=1000, type=int, help='valid/test log every xx steps')

    parser.add_argument('--no_decay', action='store_true', help='Learning rate do not decay')
    return parser.parse_args(args)


def override_config(args):
    '''
    Override model and data configuration
    '''

    with open(os.path.join(args.init_checkpoint, 'config.json'), 'r') as f:
        args_dict = json.load(f)

    args.model = args_dict['model']
    args.data_path = args_dict['data_path']
    # args.data_path = '/home/yf/code/KGE-HAKE-master/data/F2F-V2/entities.dict'  # yf
    args.hidden_dim = args_dict['hidden_dim']
    args.test_batch_size = args_dict['test_batch_size']

def model_choice(chosen_log):

    ###########################
    # Call the test initializer
    ###########################

    # Automatically retrieve the last trained model
    if chosen_log in ['last_ModelNet40', 'last_ShapeNetPart', 'last_S3DIS']:

        # Dataset name
        test_dataset = '_'.join(chosen_log.split('_')[1:])

        # List all training logs
        logs = np.sort([os.path.join('results', f) for f in os.listdir('results') if f.startswith('Log')])

        # Find the last log of asked dataset
        for log in logs[::-1]:
            log_config = Config()
            log_config.load(log)
            if log_config.dataset.startswith(test_dataset):
                chosen_log = log
                break

        if chosen_log in ['last_ModelNet40', 'last_ShapeNetPart', 'last_S3DIS']:
            raise ValueError('No log of the dataset "' + test_dataset + '" found')

    # Check if log exists
    if not os.path.exists(chosen_log):
        raise ValueError('The given log does not exists: ' + chosen_log)

    return chosen_log

def save_model(model, optimizer, save_variable_list, args):
    '''
    Save the parameters of the model and the optimizer,
    as well as some other variables such as step and learning_rate
    '''

    args_dict = vars(args)
    with open(os.path.join(args.save_path, 'config.json'), 'w') as f:
        json.dump(args_dict, f, indent=4)

    torch.save({
        **save_variable_list,
        'model_state_dict': model.state_dict(),
        'optimizer_state_dict': optimizer.state_dict()},
        os.path.join(args.save_path, 'checkpoint')
    )

    entity_embedding = model.entity_embedding.detach().cpu().numpy()
    np.save(
        os.path.join(args.save_path, 'entity_embedding'),
        entity_embedding
    )

    relation_embedding = model.relation_embedding.detach().cpu().numpy()
    np.save(
        os.path.join(args.save_path, 'relation_embedding'),
        relation_embedding
    )
# ---------yf-------------------
def find_grasstype_values(filename, grasstype):
    with open(filename, 'r', encoding='utf-8') as file:
        for line in file:
            # 分割每行数据
            parts = line.strip().split(',')
            # 检查是否是所需的grasstype
            if parts[0] == grasstype:
                # 返回找到的六维值
                return parts[1:]
    # 如果没有找到匹配的grasstype
    return None
# ---------yf-------------------
def set_logger(args):
    '''
    Write logs to checkpoint and console
    '''

    if args.do_train:
        log_file = os.path.join(args.save_path or args.init_checkpoint, 'train.log')
    else:
        log_file = os.path.join(args.save_path or args.init_checkpoint, 'test.log')

    logging.basicConfig(
        format='%(asctime)s %(levelname)-8s %(message)s',
        level=logging.INFO,
        datefmt='%Y-%m-%d %H:%M:%S',
        filename=log_file,
        filemode='w'
    )
    console = logging.StreamHandler()
    console.setLevel(logging.INFO)
    formatter = logging.Formatter('%(asctime)s %(levelname)-8s %(message)s')
    console.setFormatter(formatter)
    logging.getLogger('').addHandler(console)


def log_metrics(mode, step, metrics):
    '''
    Print the evaluation logs
    '''
    for metric in metrics:
        logging.info('%s %s at step %d: %f' % (mode, metric, step, metrics[metric]))

def extract_values_to_tensor(input_folder, graspindex):
    # Map graspindex to file number (assuming 'fingertype_0' corresponds to the first file)
    file_number = int(graspindex.split('_')[-1])

    # Construct the file path
    file_path = os.path.join(input_folder, f'grasptype_{file_number}.xml')

    with open(file_path, 'r') as file:
        content = file.read().strip()

        # Extract the values
    values_list = [float(value) for value in content.split()]

    # Create a tensor from the values
    tensor = np.array(values_list).reshape(1, 18)

    return tensor

def finger_cor(input_points, input_features):
    # 找出第 4 个特征值为 1 的索引
    index_feature_indices = np.where(input_features[:, 3] == 1)[0]
    # 获取这些索引对应的 input_points 坐标
    index_cor = input_points[index_feature_indices]

    # 找出第 1 个特征值为 1 的索引
    thumb_feature_indices = np.where(input_features[:, 0] == 1)[0]
    # 获取这些索引对应的 input_points 坐标
    thumb_cor = input_points[thumb_feature_indices]
    return thumb_cor, index_cor


def main(args):
    if (not args.do_train) and (not args.do_valid) and (not args.do_test):
        raise ValueError('one of train/val/test mode must be choosed.')

    if args.init_checkpoint:
        override_config(args)
    elif args.data_path is None:
        raise ValueError('one of init_checkpoint/data_path must be choosed.')

    if args.do_train and args.save_path is None:
        raise ValueError('Where do you want to save your trained model?')

    if args.save_path and not os.path.exists(args.save_path):
        os.makedirs(args.save_path)

    # Write logs to checkpoint and console
    set_logger(args)

    # data_reader = DataReader(args.data_path)
    data_reader = DataReader('/home/yf/code/KGE-HAKE-master/data/F2F-V2')
    num_entity = len(data_reader.entity_dict)
    num_relation = len(data_reader.relation_dict)

    logging.info('Model: {}'.format(args.model))
    logging.info('Data Path: {}'.format(args.data_path))
    logging.info('Num Entity: {}'.format(num_entity))
    logging.info('Num Relation: {}'.format(num_relation))

    logging.info('Num Train: {}'.format(len(data_reader.train_data)))
    logging.info('Num Valid: {}'.format(len(data_reader.valid_data)))
    logging.info('Num Test: {}'.format(len(data_reader.test_data)))

    if args.model == 'ModE':
        kge_model = ModE(num_entity, num_relation, args.hidden_dim, args.gamma)
    elif args.model == 'HAKE':
        kge_model = HAKE(num_entity, num_relation, args.hidden_dim, args.gamma, args.modulus_weight, args.phase_weight)

    logging.info('Model Parameter Configuration:')
    for name, param in kge_model.named_parameters():
        logging.info('Parameter %s: %s, require_grad = %s' % (name, str(param.size()), str(param.requires_grad)))

    kge_model = kge_model.cuda()

    if args.do_train:
        # Set training dataloader iterator
        train_dataloader_head = DataLoader(
            TrainDataset(data_reader, args.negative_sample_size, BatchType.HEAD_BATCH),
            batch_size=args.batch_size,
            shuffle=True,
            num_workers=max(1, args.cpu_num // 2),
            collate_fn=TrainDataset.collate_fn
        )

        train_dataloader_tail = DataLoader(
            TrainDataset(data_reader, args.negative_sample_size, BatchType.TAIL_BATCH),
            batch_size=args.batch_size,
            shuffle=True,
            num_workers=max(1, args.cpu_num // 2),
            collate_fn=TrainDataset.collate_fn
        )

        train_iterator = BidirectionalOneShotIterator(train_dataloader_head, train_dataloader_tail)

        # Set training configuration
        current_learning_rate = args.learning_rate
        optimizer = torch.optim.Adam(
            filter(lambda p: p.requires_grad, kge_model.parameters()),
            lr=current_learning_rate
        )

        warm_up_steps = args.max_steps // 2

    if args.init_checkpoint:
        # Restore model from checkpoint directory
        logging.info('Loading checkpoint %s...' % args.init_checkpoint)
        checkpoint = torch.load(os.path.join(args.init_checkpoint, 'checkpoint'))
        init_step = checkpoint['step']
        kge_model.load_state_dict(checkpoint['model_state_dict'])
        if args.do_train:
            current_learning_rate = checkpoint['current_learning_rate']
            warm_up_steps = checkpoint['warm_up_steps']
            optimizer.load_state_dict(checkpoint['optimizer_state_dict'])
    else:
        logging.info('Randomly Initializing %s Model...' % args.model)
        init_step = 0

    step = init_step

    logging.info('Start Training...')
    logging.info('init_step = %d' % init_step)
    if not args.do_test:
        logging.info('learning_rate = %d' % current_learning_rate)
    logging.info('batch_size = %d' % args.batch_size)
    logging.info('hidden_dim = %d' % args.hidden_dim)
    logging.info('gamma = %f' % args.gamma)
    logging.info('adversarial_temperature = %f' % args.adversarial_temperature)

    if args.do_train:
        training_logs = []

        # Training Loop
        for step in range(init_step, args.max_steps):

            log = kge_model.train_step(kge_model, optimizer, train_iterator, args)

            training_logs.append(log)

            if step >= warm_up_steps:
                if not args.no_decay:
                    current_learning_rate = current_learning_rate / 10
                logging.info('Change learning_rate to %f at step %d' % (current_learning_rate, step))
                optimizer = torch.optim.Adam(
                    filter(lambda p: p.requires_grad, kge_model.parameters()),
                    lr=current_learning_rate
                )
                warm_up_steps = warm_up_steps * 3

            if step % args.save_checkpoint_steps == 0:
                save_variable_list = {
                    'step': step,
                    'current_learning_rate': current_learning_rate,
                    'warm_up_steps': warm_up_steps
                }
                save_model(kge_model, optimizer, save_variable_list, args)

            if step % args.log_steps == 0:
                metrics = {}
                for metric in training_logs[0].keys():
                    metrics[metric] = sum([log[metric] for log in training_logs]) / len(training_logs)
                log_metrics('Training average', step, metrics)
                training_logs = []

            if args.do_valid and step % args.valid_steps == 0:
                logging.info('Evaluating on Valid Dataset...')
                metrics = kge_model.test_step(kge_model, data_reader, ModeType.VALID, args)
                log_metrics('Valid', step, metrics)

        save_variable_list = {
            'step': step,
            'current_learning_rate': current_learning_rate,
            'warm_up_steps': warm_up_steps
        }
        save_model(kge_model, optimizer, save_variable_list, args)

    if args.do_valid:
        logging.info('Evaluating on Valid Dataset...')
        # metrics = kge_model.test_step(kge_model, data_reader, ModeType.VALID, args)
        predition = kge_model.test_step(kge_model, data_reader, ModeType.TEST, args)  # yf
        log_metrics('Valid', step, metrics)

    if args.do_test:
        logging.info('Evaluating on Test Dataset...')
        # metrics = kge_model.test_step(kge_model, data_reader, ModeType.TEST, args)
        #------------------yf----------------------------
        metrics = kge_model.test_step1(kge_model, data_reader, ModeType.TEST, args)
        # for item in prediction:
        #     # 检查第二个元素是否是 'whichgrasptype'
        #     if item[1] == 'whichgrasptype':
        #         graspindex = item[2]
        #         graspfile = '/home/yf/code/KGE-HAKE-master/data/grasptype_yinshi'
        #         graspvalues = find_grasstype_values(graspfile, graspindex)   #  croase grasp
        #     if item[1] == 'whichfinger':
        #         fingerindex = item[2]
        #
        #     if item[1] == 'whichforce':
        #         forceindex = item[2]
        #     if item[1] == 'whichcomponent':
        #         componentindex = item[2]
        #     return graspvalues, fingerindex, forceindex, componentindex

        # ------------------yf----------------------------
        log_metrics('Test', step, metrics)


if __name__ == '__main__':
    main(parse_args())









