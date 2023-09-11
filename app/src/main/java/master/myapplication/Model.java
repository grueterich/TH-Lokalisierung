package master.myapplication;
/*import torch
from torch import nn

class Model(nn.Module):
        def __init__(self):
        super(Model, self).__init__()

        # Basic MLP with 2 inputs, 4 hidden layers
        # and 10 outputs where each output is
        # the softmax probabilities of a number 0 to 9
        self.MLP = nn.Sequential(
        nn.Linear(2, 5),
        nn.Linear(5, 10),
        nn.Linear(10, 15),
        nn.Linear(15, 20),
        nn.Linear(20, 15),
        nn.Linear(15, 10),
        nn.Softmax(-1)
        )

        def forward(self, X):
        return torch.argmax(self.MLP(X), dim=-1)*/