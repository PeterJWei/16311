function [ counter, seg_im ] = segmentor( colors, binary_image )
m = size(binary_image,1);
n = size(binary_image,2);
seg_im = binary_image;

seg_im(:,:,1) = (seg_im(:,:,1) > .5);
seg_im(:,:,2) = (seg_im(:,:,1) > .5);
seg_im(:,:,3) = (seg_im(:,:,1) > .5);

%for i = 1:m
%    for j = 1:n
%        if (seg_im(i,j,1) < .5)
%            seg_im(i,j,1) = 0;
%            seg_im(i,j,2) = 0;
%            seg_im(i,j,3) = 0; %clamp any excess noise to black
%        else
%            seg_im(i,j,1) = 1;
%            seg_im(i,j,2) = 1;
%            seg_im(i,j,3) = 1; %clamp bright pixels to white
%        end
%    end
%end
counter = 2;
for i = 1:m
    for j = 1:n
        if(seg_im(i,j,1) > .99 && seg_im(i,j,2) > .99 && seg_im(i,j,3) > .99)
            seg_im = extract_segment(colors, seg_im,i,j, counter);
            counter = counter + 1;
        end
    end
end


end


function [seg_im] = extract_segment(colors, im, sy, sx, counter)
   seg = [sy, sx];
   seg_im = im;
   [sm,sn] = size(im);
   working = true;
   while(working)
       [m, ~] = size(seg);
       for i = 1:m
           n = seg(i,:);
           x = n(2);
           y = n(1);
           if(x + 1 <= sn && seg_im(y,x+1,1) > .99 && seg_im(y,x+1,2) > .99 && seg_im(y,x+1,3) > .99)
               seg_im(y,x+1,1) = colors(mod(counter-1,8)+1,1);
               seg_im(y,x+1,2) = colors(mod(counter-1,8)+1,2);
               seg_im(y,x+1,3) = colors(mod(counter-1,8)+1,3);
               seg = [seg ; [y, x+1]];
           end
           if(x - 1 > 0 && seg_im(y,x-1,1) > .99 && seg_im(y,x-1,2) > .99 && seg_im(y,x-1,3) > .99)
               seg_im(y,x-1,1) = colors(mod(counter-1,8)+1,1);
               seg_im(y,x-1,2) = colors(mod(counter-1,8)+1,2);
               seg_im(y,x-1,3) = colors(mod(counter-1,8)+1,3);
               seg = [seg ; [y, x-1]];
           end
           if(y + 1 <= sm && seg_im(y+1,x,1) > .99 && seg_im(y+1,x,2) > .99 && seg_im(y+1,x,3) > .99)
               seg_im(y+1,x,1) = colors(mod(counter-1,8)+1,1);
               seg_im(y+1,x,2) = colors(mod(counter-1,8)+1,2);
               seg_im(y+1,x,3) = colors(mod(counter-1,8)+1,3);
               seg = [seg ; [y+1, x]];
           end
           if(y - 1 > 0 && seg_im(y-1,x,1) > .99 && seg_im(y-1,x,2) > .99 && seg_im(y-1,x,3) > .99)
               seg_im(y-1,x,1) = colors(mod(counter-1,8)+1,1);
               seg_im(y-1,x,2) = colors(mod(counter-1,8)+1,2);
               seg_im(y-1,x,3) = colors(mod(counter-1,8)+1,3);
               seg = [seg ; [y-1, x]];
           end
            
       
       end

       [m2, ~] = size(seg);
       if(m2 == m)
           working = false;
       end
       seg = seg(m+1:end, :);
   end
end

