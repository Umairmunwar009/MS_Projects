function  [path_node_f , path_tree_f] = smooth_data(path_node )
% This function smoothens the data by comparing distances
%   Detailed explanation goes here
n_node = [];path_tree_f=[];
d=0;dd=0;
% path_tree_f=[1 ,init(1), init(2),0,0];  %% [i x ,y ,cost, acc_dist]
n_node=path_node;

for i= 1:length(n_node(:,1))-2
    
    mid_node = [(n_node(i,1)+ n_node(i+2,1))/2 , (n_node(i,2)+n_node(i+2,2))/2];
    dd = sqrt((n_node(i,1) - n_node(i+2,1))^2 + ((n_node(i,2) - n_node(i+2,2))^2));
    d=d+dd; 
    
    path_tree_f = [path_tree_f; [i , mid_node(1), mid_node(2), dd, d ]];  %% [i x ,y ,cost, acc_dist]
        
end

figure (3)
plot(path_node(:,1),path_node(:,2),'-r');hold on
plot(path_tree_f(:,2),path_tree_f(:,3),'-g');
figure (4)
plot(path_tree_f(:,1),path_tree_f(:,5),'-g');

end

