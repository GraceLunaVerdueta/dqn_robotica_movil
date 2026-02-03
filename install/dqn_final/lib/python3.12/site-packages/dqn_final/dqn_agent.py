import numpy as np
from sklearn.neural_network import MLPRegressor
from collections import deque
import random
import pickle

class DQNAgent:

    def __init__(self,
                 state_size: int,
                 action_size: int,
                 learning_rate: float = 0.0007,
                 gamma: float = 0.99,
                 epsilon: float = 1.0,
                 epsilon_min: float = 0.05,
                 epsilon_decay: float = 0.995,
                 memory_size: int = 50000,
                 batch_size: int = 96,
                 target_update_freq: int = 80):

        self.state_size = state_size
        self.action_size = action_size
        self.gamma = gamma
        self.epsilon = epsilon
        self.epsilon_min = epsilon_min
        self.epsilon_decay = epsilon_decay
        self.batch_size = batch_size
        self.target_update_freq = target_update_freq
        self.step_count = 0

        self.memory = deque(maxlen=memory_size)

        self.q_network = MLPRegressor(
            hidden_layer_sizes=(128, 128),
            activation='relu',
            solver='adam',
            learning_rate_init=learning_rate,
            max_iter=1,
            warm_start=True,
            random_state=42
        )

        self.target_network = MLPRegressor(
            hidden_layer_sizes=(128, 128),
            activation='relu',
            solver='adam',
            learning_rate_init=learning_rate,
            max_iter=1,
            warm_start=True,
            random_state=42
        )

        dummy_X = np.random.randn(1, state_size)
        dummy_y = np.random.randn(1, action_size)
        self.q_network.fit(dummy_X, dummy_y)
        self.target_network.fit(dummy_X, dummy_y)

    def remember(self, state, action, reward, next_state, done):
        self.memory.append((state, action, reward, next_state, done))

    def act(self, state: np.ndarray, training: bool = True) -> int:
        if training and np.random.rand() <= self.epsilon:
            return random.randrange(self.action_size)

        state_reshaped = state.reshape(1, -1)
        q_values = self.q_network.predict(state_reshaped)[0]
        return np.argmax(q_values)

    def replay(self) -> float:
        if len(self.memory) < self.batch_size:
            return 0.0

        minibatch = random.sample(self.memory, self.batch_size)

        states = np.array([t[0] for t in minibatch])
        actions = np.array([t[1] for t in minibatch])
        rewards = np.array([t[2] for t in minibatch])
        next_states = np.array([t[3] for t in minibatch])
        dones = np.array([t[4] for t in minibatch])

        current_q_values = self.q_network.predict(states)
        next_q_values = self.target_network.predict(next_states)

        target_q_values = current_q_values.copy()
        for i in range(self.batch_size):
            if dones[i]:
                target_q_values[i][actions[i]] = rewards[i]
            else:
                target_q_values[i][actions[i]] = rewards[i] + self.gamma * np.max(next_q_values[i])

        self.q_network.partial_fit(states, target_q_values)

        self.step_count += 1
        if self.step_count % self.target_update_freq == 0:
            self.update_target_network()

        if self.epsilon > self.epsilon_min:
            self.epsilon *= self.epsilon_decay

        return np.mean((target_q_values - current_q_values) ** 2)

    def update_target_network(self):
        self.target_network = pickle.loads(pickle.dumps(self.q_network))

    def save(self, filepath: str):
        model_data = {
            'q_network': self.q_network,
            'target_network': self.target_network,
            'epsilon': self.epsilon,
            'step_count': self.step_count
        }
        with open(filepath, 'wb') as f:
            pickle.dump(model_data, f)

    def load(self, filepath: str):
        with open(filepath, 'rb') as f:
            model_data = pickle.load(f)
        self.q_network = model_data['q_network']
        self.target_network = model_data['target_network']
        self.epsilon = model_data['epsilon']
        self.step_count = model_data['step_count']
