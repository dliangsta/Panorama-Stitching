function im_alpha = rgb2alpha(im_input, epsilon)
% Returns the alpha channel of an RGB image.
% Pre-conditions:
%     im_input is an RGB image.
% Post-conditions:
%     im_alpha has the same size as im_input. Its intensity is between
%     epsilon and 1, inclusive.

    if nargin < 2
        epsilon = 0.001;
    end
    
    BWD = 1 - ceil(im_input);
    BWD = bwdist(BWD,'euclidean');
    for i = 1:size(BWD,3)
       BWD(:,:,i) = BWD(:,:,i)/max(max(BWD(:,:,i))); 
    end
    BWD = BWD * (1 - epsilon) + epsilon;
    im_alpha = BWD;

end