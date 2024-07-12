import numpy as np
import matplotlib.pyplot as plt
import os
import torch
import torch.nn as nn
"""
model_path
embedding_range
head_id 
tail_id 
"""

gamma_value = 9
epsilon_value = 2
hidden_dim = 1000

class EmbeddingRange(nn.Module):
    def __init__(self):
        super(EmbeddingRange, self).__init__()
        self.gamma = nn.Parameter(torch.Tensor([gamma_value]), requires_grad=False)
        self.epsilon = epsilon_value
        self.embedding_range = nn.Parameter(
            torch.Tensor([(self.gamma.item() + self.epsilon) / hidden_dim]),
            requires_grad=False
        )

embedding_range_model = EmbeddingRange()
# embedding_range = embedding_range_model.embedding_range.detach().cpu().numpy()
embedding_range = 0.028

# 加载实体嵌入

entity_embedding = np.load(os.path.join("/home/yf/code/KGE-HAKE-master/models/HAKE_F2Fnew_5", 'entity_embedding.npy'))
# self.embedding_range = nn.Parameter(
#             torch.Tensor([(self.gamma.item() + self.epsilon) / hidden_dim]),
#             requires_grad=False
#         )


with open('/home/yf/code/KGE-HAKE-master/data/F2Fnew/taskcategory', 'r') as f:
    head_ids = [int(line.strip().split()[0]) for line in f]

with open('/home/yf/code/KGE-HAKE-master/data/F2Fnew/componentcategory', 'r') as f:
    tail_ids = [int(line.strip().split()[0]) for line in f]

with open('/home/yf/code/KGE-HAKE-master/data/F2Fnew/componentclass', 'r') as f:
    componentclass_ids = [int(line.strip().split()[0]) for line in f]

with open('/home/yf/code/KGE-HAKE-master/data/F2Fnew/head_object', 'r') as f:
    object_ids = [int(line.strip().split()[0]) for line in f]
with open('/home/yf/code/KGE-HAKE-master/data/F2Fnew/taskclass', 'r') as f:
    taskclass_ids = [int(line.strip().split()[0]) for line in f]

with open('/home/yf/code/KGE-HAKE-master/data/F2Fnew/taskinstance', 'r') as f:
    taskinstance_ids = [int(line.strip().split()[0]) for line in f]

with open('/home/yf/code/KGE-HAKE-master/data/F2Fnew/objectinstance', 'r') as f:
    objectinstance_ids = [int(line.strip().split()[0]) for line in f]

# head = entity_embedding[head_ids]
# tail = entity_embedding[tail_ids]
# componentclass = entity_embedding[componentclass_ids]
object = entity_embedding[object_ids]
# taskclass = entity_embedding[taskclass_ids]
# taskinstance = entity_embedding[taskinstance_ids]
objectinstance = entity_embedding[objectinstance_ids]

# phase_head, mod_head = np.split(head, 2, axis=1)
# phase_tail, mod_tail = np.split(tail, 2, axis=1)
# phase_componentclass, mod_componentclass = np.split(componentclass, 2, axis=1)
phase_object, mod_object = np.split(object, 2, axis=1)
# phase_taskclass, mod_taskclass = np.split(taskclass, 2, axis=1)
# phase_taskinstance, mod_taskinstance = np.split(taskinstance, 2, axis=1)
phase_objectinstance, mod_objectinstance = np.split(objectinstance, 2, axis=1)

# mod_head = np.log(np.abs(mod_head))  * np.sign(mod_head)
# mod_tail = np.log(np.abs(mod_tail))  * np.sign(mod_tail)
# mod_componentclass = np.log(np.abs(mod_componentclass))  * np.sign(mod_componentclass)
mod_object = np.log(np.abs(mod_object))  * np.sign(mod_object)
# mod_taskclass = np.log(np.abs(mod_taskclass))  * np.sign(mod_taskclass)
# mod_taskinstancet = np.log(np.abs(mod_taskinstance))  * np.sign(mod_taskinstance)
mod_objectinstance = np.log(np.abs(mod_objectinstance))  * np.sign(mod_objectinstance)

# phase_head = phase_head / embedding_range * np.pi
# phase_tail = phase_tail / embedding_range * np.pi
# phase_componentclass = phase_componentclass / embedding_range * np.pi
phase_object = phase_object / embedding_range * np.pi
# phase_taskclass = phase_taskclass / embedding_range * np.pi
# phase_taskinstance = phase_taskinstance / embedding_range * np.pi
phase_objectinstance = phase_objectinstance / embedding_range * np.pi

# x_head, y_head = mod_head * np.cos(phase_head), mod_head * np.sin(phase_head)
# x_tail, y_tail = mod_tail * np.cos(phase_tail), mod_tail * np.sin(phase_tail)
# x_componentclass, y_componentclass = mod_componentclass * np.cos(phase_componentclass), mod_componentclass * np.sin(phase_componentclass)
x_object, y_object = mod_object * np.cos(phase_object), mod_object * np.sin(phase_object)
# x_taskclass, y_taskclass = mod_taskclass * np.cos(phase_taskclass), mod_taskclass * np.sin(phase_taskclass)
# x_taskinstance, y_taskinstance = mod_taskinstance * np.cos(phase_taskinstance), mod_taskinstance * np.sin(phase_taskinstance)
x_objectinstance, y_objectinstance = mod_objectinstance * np.cos(phase_objectinstance), mod_objectinstance * np.sin(phase_objectinstance)

# plt.scatter(x_head, y_head, label="taskcategory",s=5)
# plt.scatter(x_tail, y_tail, label="componentcategory", s=5)
# plt.scatter(x_componentclass, y_componentclass, label="componentclass", s=2)
plt.scatter(x_object, y_object, label="object", s=10, c='red')
# plt.scatter(x_object, y_object, label="taskclass", s=2)
# plt.scatter(x_object, y_object, label="taskinstance", s=1)
plt.scatter(x_object, y_object, label="objectinstance", s=10, c='black')
plt.axis('equal')
plt.legend()
plt.show()