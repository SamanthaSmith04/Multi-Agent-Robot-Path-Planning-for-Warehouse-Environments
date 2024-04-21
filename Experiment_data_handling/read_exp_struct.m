function [expStruct] = read_exp_struct(fileName)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
extraPrecision = 100000;
expStruct = readstruct(fileName);
expStruct.distances = reshape(expStruct.distances, expStruct.dist_size)/extraPrecision;
end

