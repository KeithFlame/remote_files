function r =normalizeRotation(r_)

    r_(1,:) = r_(1,:)/norm(r_(1,:));
    r_(2,:) = r_(2,:)/norm(r_(2,:));
    r_(3,:) = cross(r_(1,:),r_(2,:));

    r_(:,1) = r_(:,1)/norm(r_(:,1));
    r_(:,2) = r_(:,2)/norm(r_(:,2));
    r_(:,3) = cross(r_(:,1),r_(:,2));
    r = r_;
end
