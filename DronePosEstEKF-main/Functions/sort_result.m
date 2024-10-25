function [SortedDataset, SortedValues, ID] = sort_result(Dataset, choice)
    
    values = zeros(numel(Dataset),1);
   
    for index = 1:numel(Dataset)
        fieldName = sprintf('Dataset{%d}.%s', index, choice);
        values(index) = eval(fieldName);
    end
    
    [~, sortedIndices] = sort(values);

    SortedDataset = Dataset(sortedIndices);

    ID = zeros(numel(Dataset),1);
    
    for index = 1:numel(SortedDataset)
        ID(index) = str2num(SortedDataset{index}.name(22:end));
    end
    
    SortedValues = values(sortedIndices);
end