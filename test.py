import torch

beta_dist = torch.distributions.Beta(torch.tensor([2.0, 3.0]), torch.tensor([3.0, 4.0]))
print(beta_dist.sample())
print(beta_dist.log_prob(beta_dist.sample()))
