# edge-ai-assignment

## Overview

This repository contains the implementation and evaluation of parameter pruning and quantization techniques for deploying a digit classification convolutional neural network (CNN) onto resource-constrained edge devices. The project explores performance-efficiency tradeoffs by reducing the model’s size through pruning and quantization while maintaining high accuracy.

## Project Description

The objective of this project is to deploy a digit classification model on an EFM32 microcontroller. The workflow is divided into two parts:

### Part 1: Baseline Model Deployment
- **Training and Evaluation:**  
  The provided model was trained and evaluated in MATLAB, achieving an accuracy of **98.5%**.
- **Deployment:**  
  The baseline model was converted to C code using MATLAB Coder and deployed on the EFM32 board. Energy consumption was profiled over a 30+ second run, with the energy usage computed as:  


- **Memory Metrics:**  
- **RAM Usage:** 844 bytes  
- **Flash Usage:** 13,248 bytes

### Part 2: Pruned and Quantized Model Deployment
- **Model Pruning:**  
Both iterative and one-shot pruning methods were applied to the baseline model. The best pruned model (with a 90% prune ratio) achieved an accuracy of **94.8%**. Comparative plots were generated:
- Original vs. pruned accuracies vs. prune ratio.
- Pruned channels per layer vs. output channels per layer.
- **Deployment of Pruned Model:**  
The pruned model was deployed on the EFM32 board with energy usage measured at:  


- **Quantization:**  
The best pruned model was further quantized to 8-bit precision, reaching an accuracy of **94.3%** while further reducing memory requirements.

## Repository Contents

- **EdgeAI_PQ_Assignment.mlx**: MATLAB Live Script for model training, pruning, quantization, and C code generation.
- **digitsNetPredict.m**: Entry-point function for model prediction.
- **test_digitsNetPredict.m**: Test script for verifying the model prediction functionality.
- **Deployment Files**: Generated C source and header files for deployment on the EFM32 board.
- **Simplicity Studio Project Files**: Files to integrate the generated code into Simplicity Studio.

## Hardware & Software Requirements

- **Hardware:**  
- EFM32 Microcontroller (e.g., EFM32LG or EFM32GG990F1024)
- **Software:**  
- MATLAB with the following toolboxes:  
  - Statistics and Machine Learning Toolbox  
  - Embedded Coder  
  - MATLAB Coder Interface for Deep Learning  
  - Embedded Coder Support Package for ARM Cortex-M Processors  
  - GPU Coder Interface for Deep Learning (if using a GPU)  
  - Parallel Computing Toolbox (if using a GPU)
- Simplicity Studio (v4 or later) for deployment on EFM32 boards

## Setup and Deployment Instructions

1. **Clone the Repository:**
 ```bash
 git clone https://github.com/yourusername/edge-ai-pq-assignment.git
 cd edge-ai-pq-assignment
