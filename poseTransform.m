function currentPose = poseTransform(refPose, transform)
theta=refPose(3);
ct=cos(theta);
st=sin(theta);
R=[ct st
    -st ct];
currentPose(1:2)=refPose(1:2)+transform(1:2)*R;
currentPose(3)=refPose(3)+transform(3);
end
