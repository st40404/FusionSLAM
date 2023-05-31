# import gym
# import torch as th

# from stable_baselines3 import PPO

# # Custom actor (pi) and value function (vf) networks
# # of two layers of size 32 each with Relu activation function
# policy_kwargs = dict(activation_fn=th.nn.ReLU,
#                      net_arch=[dict(pi=[32, 32], vf=[32, 32])])
# # Create the agent
# model = PPO("MlpPolicy", "CartPole-v1", policy_kwargs=policy_kwargs, verbose=1)
# # Retrieve the environment
# env = model.get_env()
# # Train the agent
# model.learn(total_timesteps=100000)
# # Save the agent
# model.save("ppo_cartpole")

# del model
# # the policy_kwargs are automatically loaded
# model = PPO.load("ppo_cartpole", env=env)


import gym

from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env

# Parallel environments
vec_env = make_vec_env("CartPole-v1", n_envs=4)

model = PPO("MlpPolicy", vec_env, verbose=1)
model.learn(total_timesteps=25000)
model.save("ppo_cartpole")

del model # remove to demonstrate saving and loading

model = PPO.load("ppo_cartpole")

obs = vec_env.reset()
while True:
    action, _states = model.predict(obs)
    obs, rewards, dones, info = vec_env.step(action)
    vec_env.render("human")