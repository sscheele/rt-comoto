

## $\mathbb{POTR}$
***APPORACH***:
This paper solve the motion prediction problem in with seq2seq approach inspired by NLP tasks. The architecture proposed in the paper consists of two graph convolutional networks($\phi$ and $\psi$) and a pose transformer. In an inference process, the pose sequence is firstly fed into $\phi$ and then the encoder transformer block. The encoded sequence is then an input to decoder transformer block together with the query sequence. Finally, the predicted sequence is generated with network $\psi$ in a residual fashion. 

The aim of the $\phi$ and $\psi$ networks is to model the spatial relationships between the different elements of the body structure. And the transformer blocks should capture the spatial-temporal features of the sequence.

This architecture is trained jointly on the seq2seq task and an action classification task by adding an extra classification token to the initial pose sequence.

***SINGLE INFERENCE TIME:***
| Action | Time | 
| ----------- | ----------- | 
|directions(including initialization) | 45.91202735900879ms|
|discussion | 32.72509574890137ms|
|eating | 24.843931198120117ms|
|greeting | 20.65300941467285ms|
|phoning | 21.80314064025879ms|
|posing | 23.331165313720703ms|
|purchases | 22.153854370117188ms|
|sitting | 19.054889678955078ms|
|sittingdown | 26.460886001586914ms|
|smoking | 23.18596839904785ms|
|takingphoto | 19.637107849121094ms|
|waiting | 19.188880920410156ms|
|walking | 19.3021297454834ms|
|walkingdog | 19.80113983154297ms|
|walkingtogether | 19.545793533325195ms|

***EXPERIMENT SETUP:***
The trained model is not given in this paper. As a result model is trained with the default hyper-parameters on H3.6M dataset on an RTX3080. All the training data and checkpoints are stored in /potr/h36out, which can be viewed in tensorboard. 

The original H3.6M dataset contains 33 joints, with a initial dimensionality of $33*3=99$. In our evaluation, checkpoint 0499 is used. We consider the inference time and performance on a group of 30 sequences ($2* 15$ actions), 60 sequence($4* 15$ actions) and 120 sequences ($8* 15$ actions). 

For each sequence in the training and evaluation, only a set of 21 key joints considered. Then the sequences are trained in rotation matrix representations which makes the dimensionality of single pose $21*3*3=189$.
> _MAJOR_JOINTS = [ 0, 1, 2, 3, 4, 6, 7, 8, 9, 11, 12, 13, 14, 16, 17, 18, 19, 24, 25, 26, 27]

The model takes 50 frames, and output 25 frames of 1s motion.


***RESULTS:***
Inference time and their standard deviation are recorded in both GPU(NVIDIA GeForce RTX 3080) and CPU(Apple sillicon M1 Pro(8 cores))
1. 120 Sequences
	- time(GPU): 30(1.7) ms 
	- time(CPU): 206.9(6.5) ms
	Euler angle error
	> milliseconds     |    80 |   160 |   320 |   400 |   560 |  1000 |
		directions       | 0.198 | 0.454 | 0.758 | 0.869 | 1.017 | 1.485 |
		discussion       | 0.173 | 0.564 | 0.859 | 0.970 | 1.382 | 1.931 |
		eating           | 0.146 | 0.386 | 0.692 | 0.844 | 1.031 | 1.389 |
		greeting         | 0.293 | 0.679 | 1.167 | 1.377 | 1.698 | 1.747 |
		phoning          | 0.513 | 1.126 | 1.609 | 1.810 | 1.800 | 2.051 |
		posing           | 0.173 | 0.500 | 1.089 | 1.333 | 1.779 | 2.747 |
		purchases        | 0.337 | 0.639 | 1.066 | 1.130 | 1.484 | 2.314 |
		sitting          | 0.247 | 0.465 | 0.912 | 1.084 | 1.197 | 1.579 |
		sittingdown      | 0.256 | 0.638 | 0.996 | 1.119 | 1.291 | 1.758 |
		smoking          | 0.144 | 0.401 | 0.889 | 0.881 | 0.979 | 1.642 |
		takingphoto      | 0.123 | 0.413 | 0.711 | 0.851 | 0.973 | 1.213 |
		waiting          | 0.178 | 0.553 | 1.139 | 1.406 | 1.848 | 2.615 |
		walking          | 0.198 | 0.527 | 0.910 | 1.103 | 1.334 | 1.322 |
		walkingdog       | 0.339 | 0.801 | 1.256 | 1.434 | 1.747 | 1.982 |
		walkingtogether  | 0.180 | 0.531 | 0.853 | 0.916 | 1.040 | 1.491 |
2. 60 Sequences
- time(GPU): 32.4(1.5) ms
- time(CPU):  109.8(2.5) ms
	Euler angle error
> 	milliseconds     |    80 |   160 |   320 |   400 |   560 |  1000 |
> 	directions       | 0.151 | 0.326 | 0.680 | 0.768 | 0.931 | 1.482 |
> 	discussion       | 0.168 | 0.481 | 0.839 | 0.969 | 1.598 | 1.969 |
> 	eating           | 0.136 | 0.356 | 0.707 | 0.930 | 1.189 | 1.757 |
> 	greeting         | 0.275 | 0.731 | 1.264 | 1.494 | 1.499 | 1.516 |
> 	phoning          | 0.808 | 1.620 | 1.969 | 2.195 | 1.931 | 2.404 |
> 	posing           | 0.056 | 0.163 | 0.242 | 0.269 | 0.708 | 2.327 |
> 	purchases        | 0.403 | 0.610 | 1.014 | 1.035 | 1.236 | 2.143 |
> 	sitting          | 0.169 | 0.462 | 0.880 | 1.002 | 1.124 | 1.450 |
> 	sittingdown      | 0.191 | 0.519 | 0.921 | 1.047 | 1.251 | 1.830 |
> 	smoking          | 0.121 | 0.382 | 0.840 | 0.684 | 0.772 | 1.628 |
> 	takingphoto      | 0.143 | 0.590 | 0.890 | 1.035 | 1.046 | 1.436 |
> 	waiting          | 0.194 | 0.595 | 1.286 | 1.681 | 2.214 | 2.739 |
> 	walking          | 0.194 | 0.496 | 0.757 | 0.976 | 1.386 | 1.309 |
> 	walkingdog       | 0.276 | 0.653 | 1.104 | 1.299 | 1.396 | 1.530 |
> 	walkingtogether  | 0.191 | 0.570 | 0.896 | 0.960 | 1.080 | 1.857 |

3. 30 Sequences
- time(GPU): 32.9(1.0) ms
- time(CPU): 80.9(2.4) ms
	Euler angle error
>	milliseconds     |    80 |   160 |   320 |   400 |   560 |  1000 |
>	directions       | 0.239 | 0.498 | 1.048 | 1.114 | 1.169 | 1.458 |
>	discussion       | 0.163 | 0.464 | 1.004 | 1.295 | 2.045 | 2.798 |
>	eating           | 0.139 | 0.305 | 0.595 | 0.900 | 1.021 | 1.644 |
>	greeting         | 0.278 | 0.782 | 1.539 | 1.937 | 1.818 | 1.951 |
>	phoning          | 1.481 | 2.864 | 3.078 | 3.255 | 2.514 | 2.736 |
>	posing           | 0.031 | 0.066 | 0.159 | 0.207 | 0.444 | 1.756 |
>	purchases        | 0.341 | 0.325 | 0.421 | 0.400 | 0.279 | 0.935 |
>	sitting          | 0.145 | 0.387 | 0.831 | 0.970 | 1.174 | 1.601 |
>	sittingdown      | 0.210 | 0.645 | 0.923 | 1.039 | 1.201 | 1.807 |
>	smoking          | 0.053 | 0.142 | 0.790 | 0.474 | 0.656 | 1.618 |
>	takingphoto      | 0.147 | 0.747 | 1.085 | 1.245 | 0.966 | 1.364 |
>	waiting          | 0.140 | 0.443 | 1.040 | 1.379 | 2.002 | 2.934 |
>	walking          | 0.211 | 0.469 | 0.758 | 1.069 | 1.456 | 1.505 |
>	walkingdog       | 0.285 | 0.648 | 1.134 | 1.354 | 1.602 | 1.770 |
>	walkingtogether  | 0.197 | 0.592 | 0.887 | 0.902 | 1.023 | 1.146 |




