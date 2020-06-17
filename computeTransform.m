function transform = computeTransform (refpose,currentpose)
theta = refpose(3);
ct=cos(theta);
st=sin(theta);
R=[ct st
    -st ct];
transform(1:2) = (currentpose(1:2)-refpose(1:2))*R';
transform(3) = currentpose(3)-refpose(3);
end