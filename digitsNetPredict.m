function out = digitsNetPredict(in) %#codegen
    persistent mynet;
    if isempty(mynet)
        mynet = coder.loadDeepLearningNetwork('digitsNet_0.90_iterativePruned_params_21578.mat');
    end
    out = predict(mynet, in);
end