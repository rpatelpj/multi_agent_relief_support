function [rad, sensor_reading] = findSmallestInnerCircle(visibleMapN)
[~, n] = size(visibleMapN);
% if mod(m, 2) == 0 || mod(n, 2) == 0
%     if m > n
%         if mod(m, 2) == 0
%             % add a row of zeros
%             visibleMapN = [visibleMapN; zeros(1, n)]
%         else
%             visibleMapN = [visibleMapN zeroes(m-n, m)];
%         end
%     else if n > m
%             if mod(n, 2) == 0
%                 % add a row of zeros
%                 visibleMapN = [visibleMapN zeros(m, 1)]
%             else
%                 visibleMapN = [visibleMapN; zeroes(n-m, n)];
%             end
%         else
%             % both are equal and even i guess
%             visibleMapN = [visibleMapN zeros(m, 1)];
%             visibleMapN = [visibleMapN; zeros(1, n+1)];
%
%         end
%     end
% end

map = visibleMapN ~=0
sums = [];
% n = floor(length(visibleMapN)/2);
for ind = 1:(floor(length(visibleMapN)/2) - 1)
    vals = sum(map(1, 1:end)) + sum(map(end, 1:end)) + sum(map(2:end-1, 1)) + sum(map(2:end-1, end));
    sums = [vals sums]
    map = map(2:end-1, 2:end-1);
end
vals = sum(map(1, 1:end)) + sum(map(end, 1:end)) + sum(map(2:end-1, 1)) + sum(map(2:end-1, end));
sums = [vals sums]; % length of sums = floor(n/2)
rad = find(sums, ~0);
map = visibleMapN;
% find the closest now in that border
% have 4*(n-1) elements to choose from.
% have rad + 1 distances to check.
center = floor(n/2) + 1;
distances = [];
for i = 1:(rad+1)
    if (i == 1)
        num = i*4;
        r = [(center + rad) (center - rad)]
        c = [center center]
        inds_mat = [r c; c r]
    elseif(i == (rad + 1))
        r = [1 center + rad];
        c = [1 1 center + rad center + rad];
        inds_mat = [r r; c];
    else
        shift = i -1;
        r = [ones(1, 2)*(center-shift) ones(1, 2)*(center+shift)];
        c = [1 (center+rad) 1 (center+rad)];
        inds_mat = [r c; c r];
    end
    vals = [];
    for ind = 1:length(inds_mat)
         vals = [vals map(inds_mat(1, ind), inds_mat(2,ind))];
    end
    end
    vals = vals(vals~=0);
    if any(vals, 'all')
        sensor_reading = vals(1);
    end
    
end
