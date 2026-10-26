#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from torch.utils.tensorboard import SummaryWriter


def tensorboard_handling(start_time):
    writers = []
    # General writers
    writers.append(SummaryWriter(f"superellipsoid_learning/tensorboard_runs/{start_time}/train/"))
    writers.append(SummaryWriter(f"superellipsoid_learning/tensorboard_runs/{start_time}/valid/"))
    
    # Train writers
    categories = ['A', 'B', 'C']
    base_variables = ["x0", "y0", "z0", "n1", "n2", "ax", "ay", "az"]
    extra_variables = ["azim", "elev"]
    all_variables = base_variables + extra_variables  # Now used for all categories
    
    for category in categories:
        for var in all_variables:
            writers.append(SummaryWriter(f"superellipsoid_learning/tensorboard_runs/{start_time}/train_{var}_{category}"))
    
    for category in categories:
        for var in all_variables:
            writers.append(SummaryWriter(f"superellipsoid_learning/tensorboard_runs/{start_time}/valid_{var}_{category}"))
    
    names = ["averaged loss", "averaged loss separate"]
    return writers, names
