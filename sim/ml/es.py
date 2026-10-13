"""Estratégia evolutiva estilo OpenAI-ES: amostragem espelhada, pesos por posto, passo Adam."""
import numpy as np


class OpenAIES:
    def __init__(self, theta0, sigma=0.1, lr=0.03, pop=32, seed=0):
        self.theta = np.asarray(theta0, float).copy()
        self.sigma, self.lr, self.pop = sigma, lr, pop - pop % 2
        self.rng = np.random.default_rng(seed)
        self.m = np.zeros_like(self.theta); self.v = np.zeros_like(self.theta); self.t = 0

    def ask(self):
        half = self.rng.normal(size=(self.pop // 2, self.theta.size))
        eps = np.concatenate([half, -half])
        return self.theta + self.sigma * eps, eps

    def tell(self, scores, eps):
        scores = np.asarray(scores, float)
        ranks = np.empty_like(scores); ranks[np.argsort(scores)] = np.arange(scores.size)
        w = ranks / (scores.size - 1) - 0.5                      # posto centrado: robusto a outliers
        grad = (w[:, None] * eps).sum(0) / (scores.size * self.sigma)
        self.t += 1
        b1, b2 = 0.9, 0.999
        self.m = b1 * self.m + (1 - b1) * grad
        self.v = b2 * self.v + (1 - b2) * grad * grad
        mh = self.m / (1 - b1 ** self.t); vh = self.v / (1 - b2 ** self.t)
        self.theta += self.lr * mh / (np.sqrt(vh) + 1e-8)
        return self.theta
