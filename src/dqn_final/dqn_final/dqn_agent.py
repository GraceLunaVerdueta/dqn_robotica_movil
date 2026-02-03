import numpy as np
from sklearn.neural_network import MLPRegressor
from collections import deque
import random
import pickle


class DQNAgent:

    def __init__(self,
                 state_size,
                 action_size,
                 learning_rate=0.0008,
                 gamma=0.985,
                 epsilon=1.0,
                 epsilon_min=0.05,
                 epsilon_decay=0.995,
                 memory_size=50000,
                 batch_size=128,
                 target_update_freq=120):

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

        dummy_x = np.random.randn(1, state_size)
        dummy_y = np.random.randn(1, action_size)
        self.q_network.fit(dummy_x, dummy_y)
        self.target_network.fit(dummy_x, dummy_y)

    def remember(self, s, a, r, ns, d):
        self.memory.append((s, a, r, ns, d))

    def act(self, state, training=True):
        if training and np.random.rand() <= self.epsilon:
            return random.randrange(self.action_size)
        q = self.q_network.predict(state.reshape(1, -1))[0]
        return np.argmax(q)

    def replay(self):
        batch = random.sample(self.memory, self.batch_size)

        s = np.array([b[0] for b in batch])
        a = np.array([b[1] for b in batch])
        r = np.array([b[2] for b in batch])
        ns = np.array([b[3] for b in batch])
        d = np.array([b[4] for b in batch])

        q_curr = self.q_network.predict(s)
        q_next = self.target_network.predict(ns)

        q_target = q_curr.copy()
        for i in range(self.batch_size):
            if d[i]:
                q_target[i][a[i]] = r[i]
            else:
                q_target[i][a[i]] = r[i] + self.gamma * np.max(q_next[i])

        self.q_network.partial_fit(s, q_target)

        self.step_count += 1
        if self.step_count % self.target_update_freq == 0:
            self.update_target_network()

        if self.epsilon > self.epsilon_min:
            self.epsilon *= self.epsilon_decay

    def update_target_network(self):
        self.target_network = pickle.loads(pickle.dumps(self.q_network))
        print("Target network updated")

    def save(self, path):
        with open(path, 'wb') as f:
            pickle.dump({
                'q_network': self.q_network,
                'target_network': self.target_network,
                'epsilon': self.epsilon,
                'step_count': self.step_count
            }, f)
        print(f"Model saved to {path}")

    def load(self, path):
        with open(path, 'rb') as f:
            d = pickle.load(f)
        self.q_network = d['q_network']
        self.target_network = d['target_network']
        self.epsilon = d['epsilon']
        self.step_count = d['step_count']
        print(f"Model loaded from {path}")
