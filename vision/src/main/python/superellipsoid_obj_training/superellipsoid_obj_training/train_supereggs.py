#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import torch
import numpy as np
import matplotlib.pyplot as plt
import torch.nn as nn
from datetime import datetime

from . import model as mdl
from . import dataloader_supereggs as dtldr
from . import tensorboard_handling as tenshand


model_base_path = "superellipsoid_learning/model_dicts_unit_cube_rotated_simproject/"


def dataloading(object_directory, num_of_objects):

    # use 80% of objects training, 20% for validation
    train_ratio = 0.8

    # calculate the split index
    num_of_train_objects = int(num_of_objects * train_ratio)
    
    b_s = 1  # use generic batch size of 1 for now 
                    
    dataset_train = dtldr.ExrDataset(root_dir=object_directory, 
                              limit_lower=0, 
                              limit_upper=num_of_train_objects)
    
    dataset_valid = dtldr.ExrDataset(root_dir=object_directory, 
                              limit_lower=num_of_train_objects, 
                              limit_upper=num_of_objects + 1)  
   
    train_loader = torch.utils.data.DataLoader(dataset_train,
                                                batch_size=b_s,
                                                shuffle=True)
    
    valid_loader = torch.utils.data.DataLoader(dataset_valid, 
                                                batch_size=b_s, 
                                                shuffle=True)
     
    return train_loader, valid_loader


def train(num_of_objects):
    
    # Directory of the objects / their generated depth images
    object_dir = 'superellipsoid_learning/data_unit_cube_rotated_simprojc/objects/'
    device = torch.device("cpu")
   
    tao = 1e-6  # learning rate
    epochs = 5  # this will depend on how much time the user has (5 for testing now)
    len_z = 30  # length of feature vector
    
    E_img = mdl.SemanticEncoderSuperEgg(n_c=1, n_z=len_z).to(device)

    optimizer = torch.optim.Adam(E_img.parameters(), lr=tao, weight_decay=0.001)
    criterion = nn.MSELoss(reduction='mean')
    criterion_separate = torch.nn.MSELoss(reduction='none')
    
    train_dataloader, valid_dataloader = dataloading(object_dir, num_of_objects)

    # tensorboard logging 
    start_time = datetime.now().strftime("%Y.%m.%d.%H.%M.%S")
    writer, descr = tenshand.tensorboard_handling(start_time)
    
    if not os.path.exists(model_base_path + start_time):
        os.makedirs(model_base_path + start_time)
    
    # just to make sure we do not run in any issues if batches are empty
    number_of_train_images = 1e-6
    number_of_valid_images = 1e-6
    
    torch.cuda.empty_cache()

    for epoch in range(epochs):
        
        loss_avg_train = 0
        loss_avg_valid = 0
        
        loss_separate = np.zeros((len_z))

        print('/////////////////////////////////////////////////////', flush=True)
        print('Epoch  ', epoch, flush=True)
           
        num_batches_train = 1e-6
        
        for i_batch, batch in enumerate(train_dataloader):
            
            optimizer.zero_grad()
            d_img, gr_tr = batch
            gr_tr = gr_tr[:, 0, :].to(device)

            number_of_train_images = len(d_img)
            num_batches_train += 1

            z = E_img(d_img.to(device))
            z = torch.sigmoid(z)
            
            loss = criterion(z, gr_tr.to(device)) 
            loss.backward()
            optimizer.step()
            
            loss_separate += np.mean(criterion_separate(z, 
                                     gr_tr.to(device)).detach().cpu().numpy(), 
                                     axis=0) * number_of_train_images

            loss_avg_train += (loss.item() * number_of_train_images)
            
        print('train. loss is ', loss_avg_train / num_batches_train, flush=True)
        print('--------------------------------------------------', flush=True)
        writer[0].add_scalar(descr[0], loss_avg_train / num_batches_train, epoch)
        
        loss_separate = loss_separate / num_batches_train
        
        for i in range(30):  # 30 parameters
            # writers 0 and 1 are averaged training and validation loss 
            writer[2 + i].add_scalar(descr[1], np.mean(loss_separate[i]), epoch)
            
        with torch.no_grad():
            
            loss_separate = np.zeros((len_z))
            
            num_batches_valid = 1e-6
           
            for i_batch, batch in enumerate(valid_dataloader):
               
                d_img, gr_tr = batch
                gr_tr = gr_tr[:, 0, :].to(device)
                
                number_of_valid_images = len(d_img)
                num_batches_valid += 1
                
                z = E_img(d_img.to(device))
                z = torch.sigmoid(z)
                
                loss = criterion(z, gr_tr.to(device)) 
                            
                loss_separate += np.mean(criterion_separate(z, 
                                     gr_tr.to(device)).detach().cpu().numpy(), 
                                     axis=0) * number_of_valid_images

                loss_avg_valid += (loss.item() * number_of_valid_images)
                
            print('valid. loss is ', loss_avg_valid / num_batches_valid, flush=True)
            writer[1].add_scalar(descr[0], loss_avg_valid / num_batches_valid, epoch)
            
            loss_separate = loss_separate / num_batches_valid
            
            for i in range(30):  # 30 parameters
                
                writer[32 + i].add_scalar(descr[1], np.mean(loss_separate[i]), epoch)
            
            # Should we keep every model also?
            #torch.save(E_img.state_dict(),
                           #model_base_path + start_time
                               #+ "/semEncoder_%d.pt" % (epoch + 1,))
            
    # once we've finished training, export model to onnx format
    E_img.eval()       
    onnx_path = model_base_path + start_time + '/model.onnx'
    torch.onnx.export(E_img,                      # the model to be exported
                      d_img,                      # a dummy input tensor
                      onnx_path,                  # path to save the ONNX model
                      export_params=True,         # export model parameters
                      opset_version=11,           
                      do_constant_folding=True,  
                      input_names=['input'],      
                      output_names=['output']     
                    )


class TrainerWrapper:
    def __init__(self, num_of_objects=10):
        # number of objects available for training + validation
        self.num_of_objects = num_of_objects

    def run(self):
        train(self.num_of_objects)
        

if __name__ == '__main__':

    trainer = TrainerWrapper(5)
    trainer.run()

    





