# Project Overview
This project is inspired by modern Truck Platooning and Advanced Driver Assistance Systems (ADAS).
It focuses on intelligent line-following and autonomous driving concepts using machine learning, system modelling, and formal verification techniques.
The system combines:

•	SVM-based line following 

•	Scenario and requirements modelling 

•	Communication and protocol specification 

•	Timed automata and UPPAAL modelling 

•	Control behaviour design 

•	Simulation and system integration
# Objectives
The main objective is to design and analyse a simplified autonomous driving system capable of:

•	Detecting and following road lines 

•	Making steering decisions (LEFT / STRAIGHT / RIGHT) 

•	Modelling timing and communication behaviour 
# Technologies & Concepts

•	Python 

•	Scikit-learn (SVM) 

•	NumPy & Pandas 

•	Matplotlib 

•	UPPAAL 

•	Timed Automata 

•	Synthetic Image Generation 

•	Feature Extraction 

•	Machine Learning Evaluation Metrics
# Requirements Engineering
## Functional Requirements
The system must:

•	Detect lane/line position from image input 

•	Classify steering commands: (LEFT, STRAIGHT ,RIGHT) 

•	Process noisy and curved lane scenarios 

•	Simulate autonomous line-following behaviour 

•	Exchange information between system components 

•	Support timing-aware behaviour modelling 

•	Allow verification of safety-related properties
## Non-Functional Requirements
The system should provide:

•	Reliable steering prediction 

•	Real-time responsiveness 

•	Robustness against noisy sensor input 

•	Scalable communication architecture 

•	Safe and predictable control behaviour 

•	Maintainable and modular design
## Safety Requirements
The system must avoid:

•	Incorrect steering decisions 

•	Unsafe timing delays 

•	Communication failures between components 

•	Unstable control behaviour during curves 
Formal verification using UPPAAL is used to analyse and validate timing and behavioural performance
# Machine Learning Component
The project uses:

•	Synthetic camera-like images generated from CARLA

•	Feature extraction pipelines 

•	Support Vector Machines (SVMs) 

•	Minimal and extended feature spaces 

•	ROC/AUC and confusion matrix analysis 

The SVM classifier predicts steering commands based on extracted visual features.
## Feature Extraction
Two approaches are explored:
### Minimal Features
•	Bottom line offset 

•	Regional brightness features 
### Extended Features
•	Top/middle/bottom offsets 

•	Line direction 

•	Curvature estimation 

•	Brightness balance 

•	Global image statistics 
## SVM Classification
Support Vector Machines (SVMs) are used for:

•	Steering prediction 

•	Margin-based classification 

•	Nonlinear decision boundaries using RBF kernels 

Performance is evaluated using:

•	Accuracy 

•	Confusion matrices 

•	Precision / Recall 

•	F1-score 

•	ROC/AUC analysis
### Simulation and Control Behaviour
The project simulates:

•	Autonomous steering behaviour 

•	Line-following decisions 

•	Curved path handling 

•	Dynamic control responses 

The control system integrates:

•	perception 

•	classification 

•	behavioural decision-making
# Applications
Potential applications include:

•	Truck platooning 

•	ADAS systems 

•	Autonomous lane-following robots 

•	Intelligent transportation systems

