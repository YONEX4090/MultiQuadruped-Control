import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
import time
import matplotlib.pyplot as plt
import copy
import cvxpy as cp

tau = 0.05
Gamma = 0.95
state_bound = np.array([[5], [5], [5], [5]])
state_bound1 = np.array([[2], [2], [2], [2]])
Iterations_num = 10000
MaxTrials = 30
ConHor_len = 1
PreHor_len = 10
num = 1000
cost = np.zeros((num, Iterations_num))
Jsum = np.zeros((num, Iterations_num + 1))
weight_decay = 1e-8
dist_min_thres = 0.07
top_k = 12
train_steps = 4000
learning_rate = 1e-4
display_steps = 200
E = {}
U = {}
X = {}
s=0
def generate_data(num_agents, dist_min_thres):
    side_length = np.sqrt(max(1.0, num_agents / 8.0))
    states = np.zeros(shape=(num_agents, 2), dtype=np.float32)
    goals = np.zeros(shape=(num_agents, 2), dtype=np.float32)

    i = 0
    while i < num_agents:
        candidate = np.random.uniform(size=(2,)) * side_length
        dist_min = np.linalg.norm(states - candidate, axis=1).min()
        if dist_min <= dist_min_thres:
            continue
        states[i] = candidate
        i = i + 1

    i = 0
    while i < num_agents:
        candidate = np.random.uniform(-0.5, 0.5, size=(2,)) + states[i]
        dist_min = np.linalg.norm(goals - candidate, axis=1).min()
        if dist_min <= dist_min_thres:
            continue
        goals[i] = candidate
        i = i + 1

    states = np.concatenate(
        [states, np.zeros(shape=(num_agents, 2), dtype=np.float32)], axis=1)
    return states, goals


def draw_fig2Scale(ANN, NIError, State, R_State1, Obstacle, k):
    global ConHor_len, state_bound, Iterations_num, num, DesiredF, State0, E, U, X
    if k == 0:

        for i in range(1, num + 1):
            E[i] = np.zeros((4, Iterations_num))
            U[i] = np.zeros((2, Iterations_num))
            X[i] = np.zeros((4, Iterations_num))

        PresentState = State0
        R_State1 = np.array([[0], [-1], [0], [1]])
        draw_fig2Scale.PE = {}
        for i in range(1, num + 1):
            draw_fig2Scale.PE[i] = copy.deepcopy(NIError[i])

        draw_fig2Scale.Present_x1 = np.array([[0], [-1], [0], [1]])

    V = np.diag([1, 1, 0, 0])
    V1 = np.diag([0, 0, 1, 1])
    virtual = np.array([[0], [0], [0], [0]])

    timesteps = 0

    PresentError = copy.deepcopy(NIError)
    PresentState = copy.deepcopy(State)

    state_bound = np.array([[2], [2], [2], [2]])
    Umax = np.array([[1], [1]])
    Umin = np.array([[-1], [-1]])

    while timesteps < ConHor_len:
        c_input = {}
        for i in range(1, num + 1):
            c_input[i] = PresentError[i] / state_bound

        ANN[1] = NNProcess1(ANN[1], np.concatenate((c_input[1], c_input[2], c_input[num])))

        for i in range(2, num):
            ANN[i] = NNProcess1(ANN[i], np.concatenate((c_input[i], c_input[i - 1], c_input[i + 1])))

        ANN[num] = NNProcess1(ANN[num], np.concatenate((c_input[num], c_input[1], c_input[num - 1])))

        for i in range(1, num + 1):
            u[i] = copy.deepcopy(ANN[i]['NetworkOut'])
            FutureState[i] = robot(PresentState[i], u[i])
            Thita[i] = FutureState[i][2, 0]

            T[i] = np.array([[np.cos(Thita[i]), np.sin(Thita[i]), 0, 0],
                             [-np.sin(Thita[i]), np.cos(Thita[i]), 0, 0],
                             [0, 0, 1, 0],
                             [0, 0, 0, 1]])

        Future_x1 = desired_position(draw_fig2Scale.Present_x1)
        FE = {}
        FE[1] = np.dot(T[1], (Future_x1 - FutureState[1])) + np.dot(T[1], (
                FutureState[num] - FutureState[1] + (-DesiredF[:, 2 * (num - 1) - 1]).reshape(-1, 1)))

        for i in range(2, num):
            FE[i] = np.dot(T[i],
                           np.dot(V, (FutureState[i - 1] - FutureState[i])) + DesiredF[:, 2 * (i - 1) - 2].reshape(-1,
                                                                                                                   1)) + np.dot(
                T[i], np.dot(V, (FutureState[i + 1] - FutureState[i])) + DesiredF[:, 2 * (i - 1) - 1].reshape(-1,
                                                                                                              1)) + np.dot(
                V1, (
                        R_State1 - FutureState[i]))

        FE[num] = np.dot(T[num], (
                FutureState[num - 1] - FutureState[num] + DesiredF[:, 2 * (num - 2) - 2].reshape(-1, 1))) + np.dot(
            T[num], (FutureState[1] - FutureState[num] + DesiredF[:, 2 * (num - 1) - 1].reshape(-1, 1))) + np.dot(V1, (
                R_State1 - FutureState[num]))

        for i in range(1, num + 1):
            FutureError[i] = FE[i]
            E[i][:, k + timesteps ] = draw_fig2Scale.PE[i][:, 0]
            X[i][:, k + timesteps ] = PresentState[i][:, 0]
            U[i][:, k + timesteps ] = u[i][:, 0]
            PresentError[i] = copy.deepcopy(FutureError[i])
            PresentState[i] = copy.deepcopy(FutureState[i])
            draw_fig2Scale.PE[i] = copy.deepcopy(FE[i])

        draw_fig2Scale.Present_x1 = copy.deepcopy(Future_x1)
        timesteps += 1

    for i in range(1, num + 1):
        NIError[i] = copy.deepcopy(draw_fig2Scale.PE[i])
        State[i] = copy.deepcopy(PresentState[i])

    R_State1 = copy.deepcopy(draw_fig2Scale.Present_x1)
    return NIError, State, R_State1

def generate_obstacle_circle(center, radius, num=top_k):
    theta = torch.linspace(0, np.pi * 2, steps=num, dtype=torch.float32).unsqueeze(1)
    unit_circle = torch.cat([torch.cos(theta), torch.sin(theta)], dim=1)
    circle = torch.tensor(center) + unit_circle * radius
    return circle.numpy()

def NNTrain1(NN, NNError):
    Delta2 = copy.deepcopy(NNError)
    dB1 = copy.deepcopy(Delta2)
    dW1 = np.dot(Delta2, np.tanh(NN['NetworkIn']).T)
    NN['W1'] = NN['W1'] - NN['LR'] * dW1
    NN['B1'] = NN['B1'] - NN['LR'] * dB1
    return NN


def CreateANN1(x_dim, u_dim):
    ANN = {}
    ANN['HiddenUnitNum'] = 5
    ANN['InDim'] = x_dim
    ANN['OutDim'] = u_dim
    ANN['LR'] = 0.2
    ANN['W1'] = 1 * np.random.rand(ANN['OutDim'], ANN['InDim']) - 0.5  
    ANN['B1'] = 1 * np.random.rand(ANN['OutDim'], 1) - 0.5
    return ANN


def CreateCNN1(x_dim, y_dim):
    CNN = {}
    CNN['HiddenUnitNum'] = 5
    CNN['InDim'] = x_dim
    CNN['OutDim'] = y_dim
    CNN['LR'] = 0.4
    CNN['W1'] = 1 * np.random.rand(CNN['OutDim'], CNN['InDim']) - 0.5
    CNN['B1'] = 1 * np.random.rand(CNN['OutDim'], 1) - 0.5
    return CNN


def NNProcess1(NN, input):
    NN['NetworkIn'] = copy.deepcopy(input)
    NN['NetworkOut'] = np.dot(NN['W1'], np.tanh(NN['NetworkIn'])) + NN['B1']
    return NN


def desired_position(x):
    global tau
    vr = 1
    wr = 0
    FutureState = np.zeros((4, 1))
    FutureState[0, 0] = x[0, 0] + tau * vr * np.cos(x[2, 0])
    FutureState[1, 0] = x[1, 0] + tau * vr * np.sin(x[2, 0])
    FutureState[2, 0] = x[2, 0] + tau * wr
    FutureState[3, 0] = vr
    if FutureState[2, 0] > np.pi:
        FutureState[2, 0] = FutureState[2, 0] - 2 * np.pi
    elif FutureState[2, 0] < -np.pi:
        FutureState[2, 0] = FutureState[2, 0] + 2 * np.pi
    Future_x = np.array([FutureState[0, 0], FutureState[1, 0], FutureState[2, 0], FutureState[3, 0]]).reshape(-1, 1)
    return Future_x


def robot(x, u):
    global tau

    v = x[3, 0] + tau * u[0, 0]
    FutureState = np.zeros((4, 1))
    FutureState[0, 0] = x[0, 0] + tau * v * np.cos(x[2, 0])
    FutureState[1, 0] = x[1, 0] + tau * v * np.sin(x[2, 0])
    FutureState[2, 0] = x[2, 0] + tau * u[1, 0]
    FutureState[3, 0] = v
    if FutureState[2, 0] > np.pi:
        FutureState[2, 0] = FutureState[2, 0] - 2 * np.pi
    elif FutureState[2, 0] < -np.pi:
        FutureState[2, 0] = FutureState[2, 0] + 2 * np.pi
    return FutureState


def sys_process_four(ep, u, PresentState1, PresentState2, PresentState4):
    global tau

    ep1 = ep[0, 0]
    ep2 = ep[1, 0]
    ep3 = ep[2, 0]
    ep4 = ep[3, 0]
    u1 = u[0, 0]
    u2 = u[1, 0]
    wr = 0
    vr = 1
    v4 = PresentState4[3, 0]
    v2 = PresentState2[3, 0]
    thita2 = PresentState2[2, 0]
    thita1 = PresentState1[2, 0]
    thita4 = PresentState4[2, 0]
    MSJ1 = np.array([[1, tau * u2,
                      -tau * np.sin(ep3) * vr - tau * np.sin(thita2 - thita1) * v2 - tau * np.sin(thita4 - thita1) * v4,
                      2 * tau],
                     [-tau * u2, 1,
                      tau * np.cos(ep3) * vr + tau * np.cos(thita2 - thita1) * v2 + tau * np.cos(thita4 - thita1) * v4,
                      0],
                     [0, 0, 1, 0],
                     [0, 0, 0, 1]])

    MCJ1 = np.array([[0, ep2 * tau],
                     [0, -ep1 * tau],
                     [0, -tau],
                     [-tau, 0]])

    MSJ12 = np.array([[0, 0, tau * np.sin(thita2 - thita1) * v2, -tau * np.cos(thita2 - thita1)],
                      [0, 0, -tau * np.cos(thita2 - thita1) * v2, -tau * np.sin(thita2 - thita1)],
                      [0, 0, 0, 0],
                      [0, 0, 0, 0]])

    MSJ14 = np.array([[0, 0, tau * np.sin(thita4 - thita1) * v4, -tau * np.cos(thita4 - thita1)],
                      [0, 0, -tau * np.cos(thita4 - thita1) * v4, -tau * np.sin(thita4 - thita1)],
                      [0, 0, 0, 0],
                      [0, 0, 0, 0]])

    return MSJ1, MCJ1, MSJ12, MSJ14

def generate_obstacle_rectangle(center, sides, num=top_k):
    a, b = sides
    n_side_1 = int(num // 2 * a / (a + b))
    n_side_2 = num // 2 - n_side_1
    n_side_3 = n_side_1
    n_side_4 = num - n_side_1 - n_side_2 - n_side_3

    side_1 = torch.stack([torch.linspace(-a/2, a/2, n_side_1, dtype=torch.float32), torch.full((n_side_1,), b/2)], dim=1)
    side_2 = torch.stack([torch.full((n_side_2,), a/2), torch.linspace(b/2, -b/2, n_side_2, dtype=torch.float32)], dim=1)
    side_3 = torch.stack([torch.linspace(a/2, -a/2, n_side_3, dtype=torch.float32), torch.full((n_side_3,), -b/2)], dim=1)
    side_4 = torch.stack([torch.full((n_side_4,), -a/2), torch.linspace(-b/2, b/2, n_side_4, dtype=torch.float32)], dim=1)

    rectangle = torch.cat([side_1, side_2, side_3, side_4], dim=0)
    rectangle = rectangle + torch.tensor(center, dtype=torch.float32)
    return rectangle.numpy()


def dynamics(s, a):
    return torch.cat([s[:, 2:], a], dim=1)

class NetworkCBF(nn.Module):
    def __init__(self):
        super(NetworkCBF, self).__init__()
        self.conv1 = nn.Conv1d(top_k, 64, kernel_size=1)
        self.conv2 = nn.Conv1d(64, 128, kernel_size=1)
        self.conv3 = nn.Conv1d(128, 64, kernel_size=1)
        self.conv4 = nn.Conv1d(64, 1, kernel_size=1)

    def forward(self, x, mask):
        x = F.relu(self.conv1(x))
        x = F.relu(self.conv2(x))
        x = F.relu(self.conv3(x))
        x = self.conv4(x)
        return x * mask


class NetworkAction(nn.Module):
    def __init__(self):
        super(NetworkAction, self).__init__()
        self.conv1 = nn.Conv1d(top_k, 64, kernel_size=1)
        self.conv2 = nn.Conv1d(64, 128, kernel_size=1)
        self.fc1 = nn.Linear(128 + 4, 64)
        self.fc2 = nn.Linear(64, 128)
        self.fc3 = nn.Linear(128, 64)
        self.fc4 = nn.Linear(64, 4)

    def forward(self, s, g, mask):
        x = F.relu(self.conv1(s))
        x = F.relu(self.conv2(x))
        x = (x * mask).max(dim=2).values
        x = torch.cat([x, s[:, :2] - g, s[:, 2:]], dim=1)
        x = F.relu(self.fc1(x))
        x = F.relu(self.fc2(x))
        x = F.relu(self.fc3(x))
        x = self.fc4(x)
        x = 2.0 * torch.sigmoid(x) + 0.2
        return x


def loss_actions(s, g, a):
    state_gain = -torch.eye(2, 4) - torch.eye(2, 4, 2) * np.sqrt(3)
    s_ref = torch.cat([s[:, :2] - g, s[:, 2:]], dim=1)
    action_ref = torch.mm(s_ref, state_gain.t())
    norm_diff = torch.abs(a.norm(dim=1)**2 - action_ref.norm(dim=1)**2)
    return norm_diff.mean()


def train(model_cbf, model_action, optimizer_cbf, optimizer_action, num_agents, num_epochs=train_steps):
    for epoch in range(num_epochs):
        states, goals = generate_data(num_agents, dist_min_thres)
        states = torch.tensor(states, dtype=torch.float32)
        goals = torch.tensor(goals, dtype=torch.float32)

        optimizer_cbf.zero_grad()
        optimizer_action.zero_grad()
        optimizer_cbf.step()
        optimizer_action.step()

# Initialization
ANN = {}
CNN = {}

R_State1 = np.array([[0], [-1], [0], [1]])

for i in range(1, num + 1):
    ANN[i] = CreateANN1(12, 2)
    CNN[i] = CreateCNN1(12, 12)

State = {}
Thita = {}
T = {}

for i in range(1, num + 1):
    if i >= 1 and i <= 0.5 * num:
        State[i] = np.array([[-2 * (i - 1) + 1 * np.random.rand(1)[0]], [-1], [0], [1]])
        # State[i] = np.array([[-2 * (i - 1) + 0.5], [-1], [0], [1]])
    else:
        State[i] = np.array([[2 * (i - num) + 1 * np.random.rand(1)[0]], [1], [0], [1]])
        # State[i] = np.array([[2 * (i - num) + 0.5], [1], [0], [1]])
    Thita[i] = State[i][2, 0]
    T[i] = np.array([[np.cos(Thita[i]), np.sin(Thita[i]), 0, 0],
                     [-np.sin(Thita[i]), np.cos(Thita[i]), 0, 0],
                     [0, 0, 1, 0],
                     [0, 0, 0, 1]])

V = np.diag([1, 1, 0, 0])
V1 = np.diag([0, 0, 1, 1])

DF = np.hstack((np.array([[-2], [0], [0], [0]]), np.zeros((4, 2 * (num - 1) - 1))))
DesiredF = np.array([[-2], [0], [0], [0]])
for i in range(1, 2 * (num - 1)):
    if i < num - 3 or (i > num - 3 and i != num - 1 and i < 2 * (num - 1) - 1):
        DF[:, i] = -DF[:, i - 1]
    elif i == num - 3 or i == 2 * (num - 1) - 1:
        DF[:, i] = np.hstack(([[DF[1, i - 1]], [DF[0, i - 1]], [0], [0]]))
    elif i == num - 1:
        DF[:, i] = np.hstack((-1 * np.array([[DF[1, i - 1]], [DF[0, i - 1]], [0], [0]])))
    DesiredF = np.hstack((DesiredF, DF[:, i].reshape(-1, 1)))

NIError = {}
NIError[1] = T[1].dot(R_State1 - State[1]) + T[1].dot(
    State[num] - State[1] + (-DesiredF[:, 2 * (num - 1) - 1]).reshape(-1, 1))

for i in range(2, num):
    NIError[i] = T[i].dot(V.dot(State[i - 1] - State[i]) + DesiredF[:, 2 * (i - 1) - 2].reshape(-1, 1)) + T[i].dot(
        V.dot(State[i + 1] - State[i]) + DesiredF[:, 2 * (i - 1) - 1].reshape(-1, 1)) + V1.dot(R_State1 - State[i])

NIError[num] = T[num].dot(State[num - 1] - State[num] + DesiredF[:, 2 * (num - 1) - 2].reshape(-1, 1)) + T[num].dot(
    State[1] - State[num] + DesiredF[:, 2 * (num - 1) - 1].reshape(-1, 1)) + V1.dot(R_State1 - State[num])

scale = 0.01
mu = 0.001
Q = 1 * np.eye(12) * scale
R = 0.5 * np.eye(2) * scale
J = []
virtual = np.array([[0], [0], [0], [0]])
Umax = np.array([[1], [1]])
Umin = np.array([[-1], [-1]])
Obstacle = np.array([[30], [1]])
OO = np.zeros((4, 4))

P1 = np.array([[148.133190596417, 2.02583848679917, 3.51323846649974, 46.7730691661438],
               [2.02583848679917, 148.133191490445, 46.7730693299270, 3.51323630780372],
               [3.51323846649974, 46.7730693299270, 92.6842566525951, 8.80444258168160],
               [46.7730691661438, 3.51323630780372, 8.80444258168160, 92.6842564186466]])

P2 = np.array([[148.133190600772, 2.02583848790635, 3.51323846727139, 46.7730691676650],
               [2.02583848790635, 148.133191490058, 46.7730693308098, 3.51323630784338],
               [3.51323846727139, 46.7730693308098, 92.6842566511360, 8.80444258118404],
               [46.7730691676650, 3.51323630784338, 8.80444258118404, 92.6842564014894]])

P3 = np.array([[658.124851396075, 11.5177222865393, 18.0575925561011, 244.738183532405],
               [11.5177222865393, 658.124847239358, 244.738181009361, 18.0575821660109],
               [18.0575925561011, 244.738181009361, 213.345125351539, 21.9218553649452],
               [244.738183532405, 18.0575821660109, 21.9218553649452, 213.345126399689]])

P4 = np.array([[888.158317116642, 14.8674148692541, 21.2150995166181, 285.920616563882],
               [14.8674148692541, 888.158303683769, 285.920611242892, 21.2150929470356],
               [21.2150995166181, 285.920611242892, 214.761632207292, 22.3319105649926],
               [285.920616563882, 21.2150929470356, 22.3319105649926, 214.761634284998]])

P5 = np.array([[888.158317389190, 14.8674148497524, 21.2150995112752, 285.920616599051],
               [14.8674148497524, 888.158303732463, 285.920611276001, 21.2150929405337],
               [21.2150995112752, 285.920611276001, 214.761632233751, 22.3319105640493],
               [285.920616599051, 21.2150929405337, 22.3319105640493, 214.761634279374]])

P6 = np.array([[658.124851366556, 11.5177222796616, 18.0575925467624, 244.738183446075],
               [11.5177222796616, 658.124847194266, 244.738180976648, 18.0575821612168],
               [18.0575925467624, 244.738180976648, 213.345125331307, 21.9218553602577],
               [244.738183446075, 18.0575821612168, 21.9218553602577, 213.345126344712]])

P = {}
P[1] = scale * P1
P[2] = scale * P2
P[3] = scale * P3
P[4] = scale * P4
P[5] = scale * P5
P[6] = scale * P6

if num > 6:
    for i in range(6, num + 1):
        P[i] = scale * P6

run = 1
R_State10 = copy.deepcopy(R_State1)
State0 = copy.deepcopy(State)

NIError0 = copy.deepcopy(NIError)

J = np.zeros((run, Iterations_num))
tic = time.time()

for iter in range(run):
    # print(f"run={iter}")
    R_State1 = copy.deepcopy(R_State10)

    NIError = copy.deepcopy(NIError0)

    State = copy.deepcopy(State0)
    for k in range(0, Iterations_num, ConHor_len):
        Present_rx1 = copy.deepcopy(R_State1)

        RealError = copy.deepcopy(NIError)
        RealState = copy.deepcopy(State)
        f = 1
        while f >= 1:
            Present_x1 = copy.deepcopy(Present_rx1)
            PresentError = copy.deepcopy(RealError)
            PresentState = copy.deepcopy(RealState)
            Err1 = 0
            Err2 = 0
            timesteps = 0
            j = 0
            c_input = {}
            u = {}
            FutureState = {}
            MJ = {}
            FutureError = {}
            f_input = {}
            dR_dZ = {}
            FutureLambda = {}
            PresentLambda = {}
            ANNError = {}
            CNNTarget = {}

            CNNError = {}
            if k % display_steps == 0:
                print(f"time step={k}")

            while max(abs(PresentError[1]) - state_bound) <= 0 and timesteps < PreHor_len:

                for i in range(1, num + 1):
                    c_input[i] = PresentError[i] / state_bound1

                ANN[1] = NNProcess1(ANN[1], np.concatenate(
                    (c_input[1].reshape(-1, 1), c_input[2].reshape(-1, 1), c_input[num].reshape(-1, 1))))
                if num > 2:
                    for i in range(2, num):
                        ANN[i] = NNProcess1(ANN[i], np.concatenate(
                            (c_input[i].reshape(-1, 1), c_input[i - 1].reshape(-1, 1), c_input[i + 1].reshape(-1, 1))))

                ANN[num] = NNProcess1(ANN[num],
                                      np.concatenate((c_input[num].reshape(-1, 1), c_input[1].reshape(-1, 1),
                                                      c_input[num - 1].reshape(-1, 1))))


                for i in range(1, num + 1):
                    u[i] = copy.deepcopy(ANN[i]['NetworkOut'])
                    FutureState[i] = robot(PresentState[i], u[i])

                Future_x1 = desired_position(Present_x1)


                MJ[1, 1], MJ[1, 2], MJ[1, 3], MJ[1, 4] = sys_process_four(PresentError[1], u[1], PresentState[1],
                                                                          PresentState[num], PresentState[2])
                if num > 2:
                    for i in range(2, num):
                        MJ[i, 1], MJ[i, 2], MJ[i, 3], MJ[i, 4] = sys_process_four(PresentError[i], u[i],
                                                                                  PresentState[i], PresentState[i - 1],
                                                                                  PresentState[i + 1])

                MJ[num, 1], MJ[num, 2], MJ[num, 3], MJ[num, 4] = sys_process_four(PresentError[num],
                                                                                  u[num],
                                                                                  PresentState[num],
                                                                                  PresentState[num - 1],
                                                                                  PresentState[1])

                for i in range(1, num + 1):
                    Thita[i] = FutureState[i][2, 0]
                    T[i] = np.array([[np.cos(Thita[i]), np.sin(Thita[i]), 0, 0],
                                     [-np.sin(Thita[i]), np.cos(Thita[i]), 0, 0],
                                     [0, 0, 1, 0],
                                     [0, 0, 0, 1]])


                FutureError[1] = np.dot(T[1], (Future_x1 - FutureState[1])) + np.dot(T[1], (
                        FutureState[num] - FutureState[1]) + (-DesiredF[:, 2 * (num - 1) - 1]).reshape(-1, 1))

                if num > 2:
                    for i in range(2, num):
                        FutureError[i] = np.dot(T[i], (
                                V @ (FutureState[i - 1] - FutureState[i]) + DesiredF[:, 2 * (i - 1) - 2].reshape(-1,
                                                                                                                 1))) + np.dot(
                            T[i], (V @ (FutureState[i + 1] - FutureState[i]) + DesiredF[:, 2 * (i - 1) - 1].reshape(-1,
                                                                                                                    1))) + V1 @ (
                                                 R_State1 - FutureState[i])

                FutureError[num] = np.dot(T[num], (
                        FutureState[num - 1] - FutureState[num] + DesiredF[:, 2 * (num - 1) - 2].reshape(-1,
                                                                                                         1))) + np.dot(
                    T[num],
                    (FutureState[1] - FutureState[num] + DesiredF[:, 2 * (num - 1) - 1].reshape(-1, 1))) + V1 @ (
                                           R_State1 - FutureState[num])


                for i in range(1, num + 1):
                    f_input[i] = FutureError[i] / state_bound1



                dR_dZ[1] = 2 * Q @ np.concatenate(
                    (PresentError[1].reshape(-1, 1), PresentError[2].reshape(-1, 1), PresentError[num].reshape(-1, 1)))

                if num > 2:
                    for i in range(2, num):
                        dR_dZ[i] = 2 * Q @ np.concatenate((PresentError[i].reshape(-1, 1),
                                                           PresentError[i - 1].reshape(-1, 1),
                                                           PresentError[i + 1].reshape(-1, 1)))

                dR_dZ[num] = 2 * Q @ np.concatenate((PresentError[num].reshape(-1, 1), PresentError[1].reshape(-1, 1),
                                                     PresentError[num - 1].reshape(-1, 1)))

                CNN[1] = NNProcess1(CNN[1], np.concatenate(
                    (f_input[1].reshape(-1, 1), f_input[2].reshape(-1, 1), f_input[num].reshape(-1, 1))))
                if num > 2:
                    for i in range(2, num):
                        CNN[i] = NNProcess1(CNN[i], np.concatenate(
                            (f_input[i].reshape(-1, 1), f_input[i - 1].reshape(-1, 1), f_input[i + 1].reshape(-1, 1))))

                CNN[num] = NNProcess1(CNN[num],
                                      np.concatenate((f_input[num].reshape(-1, 1), f_input[1].reshape(-1, 1),
                                                      f_input[num - 1].reshape(-1, 1))))

                if timesteps < PreHor_len - 1:
                    for i in range(1, num + 1):
                        FutureLambda[i] = copy.deepcopy(CNN[i]['NetworkOut'])
                else:

                    for i in range(1, num + 1):
                        FutureLambda[i] = 2 * np.dot(np.concatenate(
                            (np.hstack((P[i], OO, OO)), np.hstack((OO, OO, OO)), np.hstack((OO, OO, OO)))),
                            np.concatenate(
                                (FutureError[i], [[0], [0], [0], [0]], [[0], [0], [0], [0]])))

                CNN[1] = NNProcess1(CNN[1], np.concatenate((c_input[1], c_input[2], c_input[num])))
                if num > 2:
                    for i in range(2, num):
                        CNN[i] = NNProcess1(CNN[i], np.concatenate((c_input[i], c_input[i - 1], c_input[i + 1])))

                CNN[num] = NNProcess1(CNN[num], np.concatenate((c_input[num], c_input[1], c_input[num - 1])))


                for i in range(1, num + 1):
                    PresentLambda[i] = copy.deepcopy(CNN[i]['NetworkOut'])

                ANNError[1] = 2 * (R) @ u[1] + Gamma * (np.dot(MJ[1, 2].T, FutureLambda[1][0:4]) + np.dot(MJ[1, 2].T,
                                                                                                          FutureLambda[
                                                                                                              2][
                                                                                                          4:8]) + np.dot(
                    MJ[1, 2].T, FutureLambda[num][4:8]))
                # ANNError[1]=2*(R)*u[1]+mu*dB_u[1]+Gamma*(MJ[1,2]'*FutureLambda[1](1:4)+MJ[1,2]'*FutureLambda[2](5:8))
                if num > 2:
                    for i in range(2, num):
                        ANNError[i] = 2 * (R) @ u[i] + Gamma * (
                                np.dot(MJ[i, 2].T, FutureLambda[i][0:4]) + np.dot(MJ[i, 2].T, FutureLambda[i-1][
                                                                                              8:12]) + np.dot(
                            MJ[i, 2].T, FutureLambda[i + 1][4:8]))

                ANNError[num] = 2 * R @ u[num] + Gamma * (
                        np.dot(MJ[num, 2].T, FutureLambda[num][0:4]) + np.dot(MJ[num, 2].T,
                                                                              FutureLambda[1][
                                                                              8:12]) + np.dot(
                    MJ[num, 2].T, FutureLambda[num - 1][8:12]))

                for i in range(1, num + 1):
                    ANN[i] = NNTrain1(ANN[i], ANNError[i])

                Z0 = np.zeros((4, 4))
                CNNTarget[1] = dR_dZ[1] + Gamma * np.dot(np.concatenate((np.hstack(
                    (MJ[1, 1].T, MJ[2, 3].T, MJ[num, 3].T)), np.hstack((MJ[1, 3].T, MJ[2, 1].T, Z0)),
                                                                         np.hstack((MJ[1, 4].T, Z0.T, MJ[num, 1].T)))),
                    np.concatenate((FutureLambda[1][0:4], FutureLambda[2][4:8],
                                    FutureLambda[num][4:8])))
                # CNNTarget[1] = dR_dZ[1] + Gamma * np.dot(np.concatenate((MJ[1][0].T, MJ[0][3].T, MJ[2][3].T)), np.concatenate((FutureLambda[1][0:4], FutureLambda[0][8:12], FutureLambda[2][4:8])))
                if num > 2:
                    for i in range(2, num):
                        CNNTarget[i] = dR_dZ[i] + Gamma * np.dot(np.concatenate((np.hstack(
                            (MJ[i, 1].T, MJ[i - 1, 4].T, MJ[i + 1, 3].T)), np.hstack(
                            (MJ[i, 3].T, MJ[i - 1, 1].T, Z0.T)), np.hstack((MJ[i, 4].T, Z0.T, MJ[i + 1, 1].T)))),
                            np.concatenate((FutureLambda[i][0:4],
                                            FutureLambda[i - 1][8:12],
                                            FutureLambda[i + 1][4:8])))

                i = num
                CNNTarget[num] = dR_dZ[num] + Gamma * np.dot(np.concatenate((np.hstack(
                    (MJ[num, 1].T, MJ[1, 4].T, MJ[num - 1, 4].T)), np.hstack((MJ[num, 3].T, MJ[1, 1].T, Z0.T)),
                                                                             np.hstack((MJ[num, 4].T, Z0,
                                                                                        MJ[num - 1, 1].T)))),
                    np.concatenate((
                        FutureLambda[i][0:4], FutureLambda[1][8:12],
                        FutureLambda[i - 1][8:12])))

                for i in range(1, num + 1):
                    CNNError[i] = PresentLambda[i] - CNNTarget[i]
                    CNN[i] = NNTrain1(CNN[i], CNNError[i])
                    PresentState[i] = copy.deepcopy(FutureState[i])
                    PresentError[i] = copy.deepcopy(FutureError[i])

                Present_x1 = copy.deepcopy(Future_x1)
                timesteps = timesteps + 1
                if timesteps == 2:
                    j = j + np.dot(np.concatenate((PresentError[1], PresentError[2], PresentError[num])).T, np.dot(Q, np.concatenate((PresentError[1],
                    PresentError[2], PresentError[num])))) + np.dot(u[1].T, np.dot(R, u[1]))
                    if num > 2:
                        for i in range(2, num):
                            j = j + np.dot(
                                np.concatenate((PresentError[i], PresentError[i - 1], PresentError[i + 1])).T,
                                np.dot(Q, np.concatenate(
                                    (PresentError[i], PresentError[i - 1], PresentError[i + 1])))) + np.dot(
                                u[i].T, np.dot(R, u[i]))
                    j = j + np.dot(np.concatenate((PresentError[num], PresentError[1], PresentError[num - 1])).T,
                                   np.dot(Q, np.concatenate(
                                       (PresentError[num], PresentError[1], PresentError[num - 1])))) + np.dot(
                        u[num].T, np.dot(R, u[num]))
            # J[iter, k] = j[0, 0] / scale
            if timesteps == PreHor_len:
                NIError, State, R_State1 = draw_fig2Scale(ANN, NIError, State, R_State1, Obstacle, k)
                # print(NIError[1], '\n', State[1], '\n', R_State1, '\n')
                f = 0
            else:
                f += 1
            if f > MaxTrials:
                ANN_u = {}
                ANN_x = {}
                CNN_x = {}
                for i in range(1, num + 1):
                    ANN[i] = CreateANN1(12, 2)
                    CNN[i] = CreateCNN1(12, 12)
                f = 1

toc = time.time()
model_cbf = NetworkCBF()
model_action = NetworkAction()
optimizer_cbf = torch.optim.Adam(model_cbf.parameters(), lr=learning_rate, weight_decay=weight_decay)
optimizer_action = torch.optim.Adam(model_action.parameters(), lr=learning_rate, weight_decay=weight_decay)
train(model_cbf, model_action, optimizer_cbf, optimizer_action, num_agents=10, num_epochs=train_steps)
print(f"Time step {k}: J = {J[iter, k]}")