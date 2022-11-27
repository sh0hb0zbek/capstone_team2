import collections
from tensorflow import keras
from keras.layers import Activation, Dense, Dropout
from keras.models import Sequential, load_model
from keras.optimizers import RMSprop
import json
import numpy as np
import os
import random
import sys
import time

import rclpy
from rclpy.node import Node

from turtlebot3_msgs.srv import Dqn


class DQNAgent(Node):
    def __init__(self):
        super().__init__('dqn_agent')
        
        """**************************
        Initialize variables
        **************************"""
        # state size and action size
        self.state_size                 = 4
        self.action_size                = 5
        self.episode_size               = 3000

        # DQN hyperparameters
        self.discount_facter            = 0.99
        self.learning_rate              = 0.00025
        self.epsilon                    = 1.0
        self.epsilon_decay              = 0.99
        self.epsilon_min                = 0.05
        self.batch_size                 = 64
        self.train_start                = 64

        # replay memory
        self.memory = collections.deque(maxlen=1_000_000)

        # build model and target model
        self.model                      = self.build_model()
        self.target_model               = self.build_model()
        self.update_target_model()
        self.update_target_model_start  = 2000

        # load saved models
        self.load_model                 = False
        self.load_episode               = 0
        self.model_dir_path             = os.path.dirname(os.path.realpath(__file__))
        self.model_dir_path             = self.model_dir_path.replace(
            'dqn',
            'model')
        self.model_path                 = os.path.join(
            self.model_dir_path,
            'episode'+str(self.load_episode) + '.h5')

        if self.load_model:
            self.model.set_weights(load_model(self.model_path).get_weights())
            with open(os.path.join(
                self.model_dir_path,
                'episode'+str(self.load_episode)+'.json')) as outfile:
                param                   = json.load(outfile)
                self.epsilon            = param.get('epsilon')
        
        """**************************
        Initialize ROS clients
        **************************"""
        self.dqn_com_client             = self.create_client(Dqn, 'dqn_com')

        """**************************
        Start the process
        **************************"""
        self.process()

    """**************************************
    Callback functions and relevant functions
    **************************************"""
    def process(self):
        global_step = 0
        
        for episode in range(self.load_episode+1, self.episode_size):
            global_step += 1
            local_step  = 0

            state = list()
            next_state = list()
            done = False
            init = True
            score = 0

            # reset DQN environment
            time.sleep(0.1)

            while not done:
                local_step += 1

                # action based on the current state
                if local_step == 1:
                    action = 2      # move forward
                else:
                    state = next_state
                    action = int(self.get_action(state))
                
                # send action and receive next state and reward
                req = Dqn.Request()
                print(int(action))
                req.action = action
                req.init = init
                while not self.dqn_com_client.wait_for_service(timeout_sec=0.1):
                    self.get_logger().info('[agent 113] service not available, waiting again ...')
                
                future = self.dqn_com_client.call_async(req)

                while rclpy.ok():
                    rclpy.spin_once(self)
                    if future.done():
                        if future.result() is not None:
                            # next state and reward
                            next_state = future.result().state
                            reward = future.result().reward
                            done = future.result().done
                            score += reward
                            init = False
                        else:
                            self.get_logger().error(
                                f'[agent 129] Exception while calling service: {future.exception()}')
                        break
                
                # save <state, action, reward, next_state> [<s, a, r, s'>]
                if local_step > 1:
                    self.append_sample(state, action, reward, next_state, done)

                    # train model
                    if global_step > self.update_target_model_start:
                        self.train_model(True)
                    elif global_step > self.train_start:
                        self.train_model()
                    
                    if done:
                        # update neural network
                        self.update_target_model()

                        print(
                            'Episode:       ',   episode,
                            '\nscore:         ', score,
                            '\nmemory length: ', len(self.memory),
                            '\nepsilon:       ', self.epsilon)
                        
                        param_keys = ['epsilon']
                        param_values = [self.epsilon]
                        param_dictionary = dict(zip(param_keys, param_values))
                # while loop rate
                time.sleep(0.01)
            
            # update result and save model every 10 episodes
            if episode % 10 == 0:
                self.model_path = os.path.join(
                    self.model_dir_path,
                    'episode'+str(episode)+'.h5')
                self.model.save(self.model_path)
                with open(os.path.join(
                    self.model_dir_path,
                    'episode'+str(episode)+'.json'), 'w') as outfile:
                    json.dump(param_dictionary, outfile)
            
            # epsilon
            if self.epsilon > self.epsilon_min:
                self.epsilon *= self.epsilon_decay
            
    def build_model(self):
        model = Sequential()
        model.add(Dense(
            64, input_shape=(self.state_size,),
            activation='relu',
            kernel_initializer='lecun_uniform'))
        model.add(Dense(
            64,
            activation='relu',
            kernel_initializer='lecun_uniform'))
        model.add(Dropout(0.2))
        model.add(Dense(
            64,
            activation='relu',
            kernel_initializer='lecun_uniform'))
        model.add(Activation('linear'))
        model.compile(
            loss='mse',
            optimizer=RMSprop(lr=self.learning_rate, rho=0.9, epsilon=1e-6))
        model.summary()

        return model
        
    def update_target_model(self):
        self.target_model.set_weights(self.model.get_weights())
    
    def get_action(self, state):
        if np.random.rand() <= self.epsilon:
            return random.randrange(self.action_size)
        else:
            state = np.asarray(state)
            q_value = self.model.predict(state.reshape(1, len(state)))
            print(np.argmax(q_value[0]))
            return np.argmax(q_value[0])
    
    def append_sample(self, state, action, reward, next_state, done):
        self.memory.append((state, action, reward, next_state, done))
    
    def train_model(self, target_train_start=False):
        mini_batch = random.sample(self.memory, self.batch_size)
        x_batch = np.empty((0, self.state_size), dtype=np.float64)
        y_batch = np.empty((0, self.action_size), dtype=np.float64)
        
        for i in range(self.batch_size):
            state       = np.asarray(mini_batch[i][0])
            action      = np.asarray(mini_batch[i][1])
            reward      = np.asarray(mini_batch[i][2])
            next_state  = np.asarray(mini_batch[i][3])
            done        = np.asarray(mini_batch[i][4])
            
            q_value = self.model.predict(state.reshape(1, len(state)))
            self.max_q_value = np.max(q_value)

            if not target_train_start:
                target_value = self.model.predict(next_state.reshape(1, len(next_state)))
            else:
                target_value = self.target_model.predict(next_state.reshape(1, len(next_state)))
            
            if done:
                next_q_value = reward
            else:
                next_q_value = reward + self.discount_facter * np.amax(target_value)
            
            x_batch = np.append(x_batch, np.array([state.copy()]), axis=0)
            
            y_sample = q_value.copy()
            y_sample[0][action] = next_q_value
            y_batch = np.append(y_batch, np.array([y_sample.copy()]), axis=0)

            if done:
                x_batch = np.array(x_batch, np.array([next_state.copy()]), axis=0)
                y_batch = np.array(y_batch, np.array([[reward]*self.action_size]), axis=0)
        
        self.model.fit(x_batch, y_batch, batch_size=self.batch_size, epochs=1, verbose=0)


def main(args=None):
    rclpy.init(args=args)
    dqn_agent = DQNAgent()
    rclpy.spin(dqn_agent)

    dqn_agent.destroy()
    rclpy.shutdown()


if __name__ == '__main__':
    main()