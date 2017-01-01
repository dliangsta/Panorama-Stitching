function [ im_blended ] = blend( im_input1, im_input2 )
%BLEND Blends two images together via feathering
% Pre-conditions:
%     `im_input1` and `im_input2` are both RGB images of the same size
%     and data type
% Post-conditions:
%     `im_blended` has the same size and data type as the input images

assert(all(size(im_input1) == size(im_input2)));
assert(size(im_input1, 3) == 3);

im_blended = zeros(size(im_input1), 'like', im_input1);

alpha1 = rgb2alpha(im_input1);
alpha2 = rgb2alpha(im_input2);
for i = 1:size(im_blended, 3)
    for j = 1:size(im_blended,1)
        for k = 1:size(im_blended,2)
            im_blended(j,k,i) = (alpha1(j,k,i)*im_input1(j,k,i) + alpha2(j,k,i)*im_input2(j,k,i)) / (alpha1(j,k,i) + alpha2(j,k,i));
        end
    end
end
imshow(im_blended);

end

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
min(min(min(im_alpha)));

end
