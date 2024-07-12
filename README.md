# Task-oriented Tool Manipulation with Robotic Dexterous Hands: A Knowledge Graph Approach from Fingers to Functionality
This is the F2F Knowledge Graph data set and code of paper **Task-oriented Tool Manipulation with Robotic Dexterous Hands: A Knowledge Graph Approach from Fingers to Functionality.** 

## Dependencies
- Python 3.6+
- [PyTorch](http://pytorch.org/) 1.0+

## Running the code 

### Usage
```
bash runs.sh {train | valid | test} {HAKE} {F2F-V2 | your Knowledge Graph} <gpu_id> \
<save_id> <train_batch_size> <negative_sample_size> <hidden_dim> <gamma> <alpha> \
<learning_rate> <num_train_steps> <test_batch_size> [modulus_weight] [phase_weight]
```
- `{ | }`: Mutually exclusive items. Choose one from them.
- `< >`: Placeholder for which you must supply a value.
- `[ ]`: Optional items.

**Remark**: `[modulus_weight]` and `[phase_weight]` are available only for the `HAKE` model.

To reproduce the results of HAKE, run the following commands.

### HAKE
```
bash runs.sh train HAKE F2F-V2 0 3 512 1024 500 12 1 0.00005 20000 16 1 1
```
## Citation
If you find this code useful, please consider citing the following paper.
```

```

## Acknowledgement
We refer to the code of [HAKE](https://github.com/MIRALab-USTC/KGE-HAKE). Thanks for their contributions.
