import torch
import torch.onnx
import torch.nn as nn
import torch.nn.functional as F
from torch.distributions import Bernoulli
from torch.autograd import Variable
import torch.optim as optim
import numpy as np
import random
import math
import sys
import os
import argparse
parser = argparse.ArgumentParser()
parser.add_argument("--seed", default = 1, type=int, help='seed for neural network')



class simpleNet(nn.Module):
    def __init__(self,  no_of_inputs = 2, no_of_outputs = 1, seed = 1):
        torch.manual_seed(seed)
        self.no_of_inputs = no_of_inputs
        self.no_of_outputs = no_of_outputs
        self.no_of_layers = 2
        self.no_of_hidden_units = 100
        super(simpleNet, self).__init__()

        self.lin_trans = []

        self.lin_trans.append(nn.Linear(self.no_of_inputs, self.no_of_hidden_units))
        self.lin_trans.append(torch.nn.ReLU())
        for layer_index in range(self.no_of_layers-2):
            self.lin_trans.append(nn.Linear(self.no_of_hidden_units, self.no_of_hidden_units))
            self.lin_trans.append(torch.nn.ReLU())

        self.lin_trans.append(nn.Linear(self.no_of_hidden_units, self.no_of_outputs))
        # self.lin_trans.append(torch.nn.ReLU())

        self.model = torch.nn.Sequential(*self.lin_trans)

    def forward(self, x):
        y = self.model(x)
        return y



if __name__ == '__main__':

    args = parser.parse_args()
    # Create a random network using pytorch
    input_count = 2
    output_count = 1
    seed = args.seed
    random_network = simpleNet(no_of_inputs = input_count, no_of_outputs = output_count, seed = seed)

    # Estimate the max and min value
    random_network.eval()

    # These are not explicitly used, but are here to make things clear
    # of course one can imagine a simple linear shift to scale the numpy
    # random numbers
    interval_1 = [0,1]
    interval_2 = [0,1]

    np.random.seed(seed)
    samples_count = 100
    random_samples = np.random.rand(samples_count, input_count).astype(np.float32)
    output_array = random_network(torch.from_numpy(random_samples))
    max = torch.max(output_array).cpu().detach().numpy()
    min = torch.min(output_array).cpu().detach().numpy()

    print("===================== Current seed - ", args.seed, " ========================== ")
    print("Python estimated interval - [", min, max,"]" )

    test_sample = np.asarray([1.0,2.0]).astype(np.float32)
    print("Test output = ", random_network(torch.from_numpy(test_sample)))
    # Creating a dummy input tensor
    dummy_input = torch.randn(1, input_count, requires_grad=True)

    # Save it in the disk by exporting the model
    torch.onnx.export(random_network, dummy_input,
         "./sherlock_python_interface/random_network.onnx", export_params=True, opset_version=9,
         do_constant_folding=True, input_names = ['modelInput'], output_names = ['modelOutput'])
