import numpy as np
import os
import sys
import json
import time
import pandas as pd
import torch
thispath = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, thispath+"/../")
import utils.utils as utils
import models.PoseEncoderDecoder as PoseEncoderDecoder
from training.transformer_model_fn import dataset_factory, POTRModelFn

_DEVICE = torch.device('cuda:0' if torch.cuda.is_available() else 'cpu')
N_ACTIONS = 15



def save_csv_eval_log(head, value, num_seed, time, is_create=False):
    if len(value.shape) < 2:
        value = np.expand_dims(value, axis=0)
    df = pd.DataFrame(value)
    file_path = '../potr_eval_{}_{}.csv'.format(15*num_seed, _DEVICE)
    if not os.path.exists(file_path) or is_create:
        df.to_csv(file_path, header=head, index=False)
    else:
        with open(file_path, 'a') as f:
            df.to_csv(f, header=False, index=False)


@torch.no_grad()
def calculate_MPJPE(prediction, target, num_seed, actions, inference_time):
    create_csv = True
    avg_ret_log = []
    pred = np.squeeze(prediction)
    tgt = np.squeeze(target)

    # T is the number of frames used here
    N, T, D = pred.shape

    # back to rotmat
    pred = pred.reshape((N_ACTIONS, num_seed, T, -1, 3, 3))
    tgt = tgt.reshape((N_ACTIONS, num_seed, T, -1, 3, 3))

    for ai, action in enumerate(actions):
        action_pred = pred[ai].cpu()
        action_tgt = tgt[ai].cpu()
        pred_exp = np.zeros((num_seed, T, 21, 3))
        tgt_exp = np.zeros((num_seed, T, 21, 3))
        for i in range(num_seed):
            for j in range(T):
                for joint in range(21):
                    pred_exp[i, j, joint] = utils.rotmat2expmap(action_pred[i, j, joint])
                    tgt_exp[i, j, joint] = utils.rotmat2expmap(action_tgt[i, j, joint])


        pred_tensor = torch.tensor(pred_exp)
        tgt_tensor = torch.tensor(tgt_exp)
        action_MPJPE = torch.sum(torch.mean(torch.norm(pred_tensor-tgt_tensor, dim=3), dim=2), dim=0)
        
        ret = {}
        for i in range(1, T+1):
            ret["#" + str(40*i) + "ms"] = torch.sum(action_MPJPE[:i]).item()
        
        ret_log = np.array([action])
        head = np.array(['action'])
        for k in ret.keys():
            ret_log = np.append(ret_log, [ret[k]])
            head = np.append(head, ['test_' + k])
        save_csv_eval_log(head=head, value=ret_log, num_seed=num_seed, time=inference_time, is_create=create_csv)
        create_csv = False
        avg_ret_log.append(ret_log[1:])

    avg_ret_log = np.array(avg_ret_log, dtype=np.float64)
    avg_ret_log = np.mean(avg_ret_log, axis=0)

    write_ret_log = ret_log.copy()
    write_ret_log[0] = 'avg'
    write_ret_log[1:] = avg_ret_log
    save_csv_eval_log(head=head, value=write_ret_log, num_seed=num_seed, time=inference_time, is_create=create_csv)
    os.rename('../potr_eval_{}_{}.csv'.format(15*num_seed, _DEVICE), './potr_eval_{}_{}_time_{}.csv'.format(15*num_seed, _DEVICE, int(inference_time)))
    
def model_factory():
    # fetch model 
    path = thispath + '/../'
    with open(path + 'h36out/config/config.json') as json_file:
        params = json.load(json_file)

    train_dataset_fn, eval_dataset_fn = dataset_factory(params)
    params['input_dim'] = train_dataset_fn.dataset._data_dim
    params['pose_dim'] = train_dataset_fn.dataset._pose_dim
    pose_encoder_fn, pose_decoder_fn = \
        PoseEncoderDecoder.select_pose_encoder_decoder_fn(params)

    model_fn = POTRModelFn(
        params, train_dataset_fn,
        eval_dataset_fn,
        pose_encoder_fn, pose_decoder_fn
    )
    print(_DEVICE)
    model_fn._model.load_state_dict(torch.load(path + "h36out/models/ckpt_epoch_0499.pt", map_location=torch.device(_DEVICE)))
    model_fn._model.eval()
    return model_fn


if __name__ == '__main__':

    with open('h36out/config/config.json') as json_file:
        params = json.load(json_file)

    train_dataset_fn, eval_dataset_fn = dataset_factory(params)
    params['input_dim'] = train_dataset_fn.dataset._data_dim
    params['pose_dim'] = train_dataset_fn.dataset._pose_dim
    pose_encoder_fn, pose_decoder_fn = \
        PoseEncoderDecoder.select_pose_encoder_decoder_fn(params)

    model_fn = POTRModelFn(
        params, train_dataset_fn,
        eval_dataset_fn,
        pose_encoder_fn, pose_decoder_fn
    )

    model_fn._model.load_state_dict(torch.load("h36out/models/ckpt_epoch_0499.pt", map_location=torch.device(_DEVICE)))
    model_fn._model.eval()

    sample = next(iter(eval_dataset_fn))
    sample_input = sample['encoder_inputs'][0]
    print(sample.keys())


    for k in sample.keys():
      if (k=='decoder_outputs_euler') or (k=='actions'):
        continue
      sample[k] = sample[k].squeeze().to(_DEVICE)
    

    encoder_sample_shape = sample['encoder_inputs'].shape
    decoder_sample_shape = sample['decoder_inputs'].shape

    print("shape of encoder input sequence: " + str(encoder_sample_shape))
    print("shape of decoder input sequence: " + str(decoder_sample_shape))
    # >>>> single inference time  
    action_time_dict = {}

    # print time csv 
    for action_id in range(params['num_activities']):
        
        single_encoder_input = sample['encoder_inputs'][action_id].view(1, encoder_sample_shape[1], encoder_sample_shape[2])
        single_decoder_input = sample['decoder_inputs'][action_id].view(1, decoder_sample_shape[1], decoder_sample_shape[2])

        start_time = time.time()
        decoder_pred = model_fn._model(
            single_encoder_input, single_decoder_input)
        inference_time = 1000*(time.time() - start_time)
        print(">>>> action: "+ str(params['action_subset'][action_id]) +  " time taken: " + str(inference_time) + "ms")


    # batch inference
    # start_time = time.time()
    # decoder_pred = model_fn._model(
    #         sample['encoder_inputs'], sample['decoder_inputs'])
    # decoder_pred = decoder_pred[0][-1]
    # inference_time = 1000*(time.time() - start_time)
    # # calculate MPJPEs and output csv 
    # calculate_MPJPE(decoder_pred, sample['decoder_outputs'], model_fn._params['eval_num_seeds'], sample['actions'], inference_time)
