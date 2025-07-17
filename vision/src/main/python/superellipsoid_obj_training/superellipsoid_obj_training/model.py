import torch.nn as nn


# Encoder which extracts geometric information.
class SemanticEncoderSuperEgg(nn.Module):
    # initializers
    def __init__(self, n_c, n_z):
        super(SemanticEncoderSuperEgg, self).__init__()
        
        # convolutional layers
        self.conv1 = nn.Conv2d(in_channels=n_c, out_channels=8, kernel_size=3, padding=1)
        self.in1 = nn.InstanceNorm2d(8)
        self.conv2 = nn.Conv2d(in_channels=8, out_channels=16, kernel_size=3, padding=1)
        self.in2 = nn.InstanceNorm2d(16)
        self.conv3 = nn.Conv2d(in_channels=16, out_channels=32, kernel_size=3, padding=1)
        self.in3 = nn.InstanceNorm2d(32)
        
        # linear layers
        self.fc1 = nn.Linear(30752, 64)  # size based on the 250 x 250 images
        self.fc2 = nn.Linear(64, n_z)
       
        self.leakyrelu = nn.LeakyReLU()
        
        
    def forward(self, x):
        # convolutional layers
        x = (self.leakyrelu(self.conv1(x)))
        x = nn.functional.max_pool2d(x, 2)
        x = (self.leakyrelu(self.conv2(x)))
        x = nn.functional.max_pool2d(x, 2)
        x = (self.leakyrelu(self.conv3(x)))
        x = nn.functional.max_pool2d(x, 2)
        
        # flatten before linear layers
        x = x.view(x.size(0), -1)
        
        # linear layers
        x = self.leakyrelu(self.fc1(x))
        z = self.fc2(x)
    
        return z

