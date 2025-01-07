import torch
print(torch.__version__)
if torch.cuda.is_available():
    print("CUDA is available. GPU will be used.")
    device = torch.device('cuda')
else:
    print("CUDA is not available. CPU will be used.")
    device = torch.device('cpu')