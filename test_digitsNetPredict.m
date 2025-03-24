%% Load sample data
[imdsTrain, imdsValidation] = loadDigitDataset;

%% Select one test sample
testData = readimage(imdsTrain, 1);
testData = reshape(testData, [28, 28, 1]); % Ensure 28x28x1 uint8

%% Test digitsNetPredict
score = digitsNetPredict(testData);

%% Define data loading function
function [imdsTrain, imdsValidation] = loadDigitDataset()
    digitDatasetPath = fullfile(matlabroot, 'toolbox', 'nnet', 'nndemos', ...
        'nndatasets', 'DigitDataset');
    imds = imageDatastore(digitDatasetPath, ...
        'IncludeSubfolders', true, 'LabelSource', 'foldernames');
    [imdsTrain, imdsValidation] = splitEachLabel(imds, 0.75, "randomized");
end