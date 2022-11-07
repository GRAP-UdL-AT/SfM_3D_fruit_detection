function images_mat=get_camera_matrices(file,images)

    cameras_data=dlmread(file,' ',1,0);
    
    images_mat=images;

    %%skip first lines
    camera_value=0;
    camera_line=0;
    while camera_value==0
        camera_line=camera_line+1;
        camera_value=cameras_data(camera_line,4);
    end
    point_line=camera_line-2;

    %start reading tie points
    disp('Reading tie points...')
    cluster_id_max=0;
    tiePoint_ID=0;
    tiePoint_data=[];

    while point_line < size(cameras_data,1)
        %disp(point_line)
        tiePoint_ID=tiePoint_ID+1;
        tiePoint_data(tiePoint_ID,:)=cameras_data(point_line,1:3);
        j=2;
        for i=1:cameras_data(point_line+2,1)    
            images(cameras_data(point_line+2,j)+1).tiePoints_IDs=[images(cameras_data(point_line+2,j)+1).tiePoints_IDs ;...
                                                                tiePoint_ID];
            images(cameras_data(point_line+2,j)+1).tiePoints_pix=[images(cameras_data(point_line+2,j)+1).tiePoints_pix ;...
                                                                  cameras_data(point_line+2,j+2)+0.5*size(images(cameras_data(point_line+2,j)+1).mask,2) ,...
                                                                  -cameras_data(point_line+2,j+3)+0.5*size(images(cameras_data(point_line+2,j)+1).mask,1)+1];
            j=j+4;
        end

        point_line=point_line+3;
    end
    
    
    
    f_waitbar = waitbar(0,strcat("Estimating camera matrices")); 
    for i=1:size(images,2)
        %disp(strcat('Camera matrix estimation: ',num2str(i)));
        waitbar(i/size(images,2),f_waitbar,strcat("Estimating camera matrix ", num2str(i), " out of ", num2str(size(images,2))));
        if size(images(i).tiePoints_pix ,1)<10
            images_mat(i).mask=zeros(size(images_mat(i).mask));
            images_mat(i).scores=[]; 
            images_mat(i).tiePoints_IDs=[];
            images_mat(i).tiePoints_pix=[];
            images_mat(i).camMatrix=zeros(4,3);
        else
%             cameraParams = estimateCameraParameters(images(i).tiePoints_pix , tiePoint_data(images(i).tiePoints_IDs,:), ...
%                                        'ImageSize',size(images_mat(i).mask));
%             [rotationMatrix,translationVector] = extrinsics( images(i).tiePoints_pix , tiePoint_data(images(i).tiePoints_IDs,:) ,cameraParams);
%             images_mat(i).camMatrix= cameraMatrix(cameraParams,rotationMatrix,translationVector);
            images_mat(i).camMatrix= estimateCameraMatrix( images(i).tiePoints_pix , tiePoint_data(images(i).tiePoints_IDs,:) );
        end
    end
    close(f_waitbar)

end



