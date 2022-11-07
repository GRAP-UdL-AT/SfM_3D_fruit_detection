function projection_visualization(images,ptCloud_xyz)

    for i=1:size(images,2)
        uv=(images(i).camMatrix'*[ptCloud_xyz(:,1:3),ones(size(ptCloud_xyz,1),1)]')';
        u=uv(:,1)./uv(:,3);
        v=uv(:,2)./uv(:,3);
        proj_img=(images(i).mask>0)*0.2;
        [H,W]=size(images(i).mask);
        for j=1:size(ptCloud_xyz,1)
            if uint16(v(j))<H && uint16(v(j))>0 && uint16(u(j))<W && uint16(u(j))>0
                proj_img(uint16(v(j)),uint16(u(j)))=1;
            end

        end
        figure;
        imshow(proj_img)
    end

end