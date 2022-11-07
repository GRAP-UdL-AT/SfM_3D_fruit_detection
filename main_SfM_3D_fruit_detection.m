clear
clc
tic
t=0;

%% Paths
addpath('lib')
cameras_file={fullfile(pwd,'data','camaras','west_side_cameras.out'), fullfile(pwd,'data','camaras','east_side_cameras.out')}; 
ptcloud_file={fullfile(pwd,'data','pt_clouds','west_side_pc.txt'), fullfile(pwd,'data','pt_clouds','east_side_pc.txt')}; 
img_mask_directory={fullfile(pwd,'data','masks','west_side_masks'),fullfile(pwd,'data','masks','east_side_masks')};
results_folder=fullfile(pwd,'results');

%% Configuration parameters
Trials={'test'};
plot_features=1;
technique_FP=2;  % 0 not FPr, 1 if FPr using nº of points and max score threshold, 2 if FPr using SVM.
KernelFunctionFP='linear';
BoxConstrainFP=0.35;
StandardizeFP=true;
KernelFunctionCCwoa='linear';
BoxConstrainCCwoa=0.35;
StandardizeCCwoa=true;
KernelFunctionCCwmtoa='linear';
BoxConstrainCCwmtoa=0.35;
StandardizeCCwmtoa=true;
k=10;
Eps=0.03;
IoD=0.5;


%% masks reading

camera_id=0;
initial_camera_id=1;

for j=1:size(img_mask_directory,2)
    [~,mask_folder_name,~]=fileparts(img_mask_directory{j});
    disp(strcat("Reading image masks (fruit detections) from: ",mask_folder_name));
    f_waitbar = waitbar(0,strcat("Processing row side ", num2str(j), " out of ", num2str(size(img_mask_directory,2)))); 
    f = dir(img_mask_directory{j});
    for i=1:size(f,1)
        if f(i).isdir==0
            if strcmp(f(i).name(end-2:end),'mat')==1
                waitbar(i/size(f,1),f_waitbar,strcat("Reading mask ", num2str(i), " out of ", num2str(size(f,1))));
                camera_id=camera_id+1;
                load(fullfile(f(i).folder,f(i).name))
                images(camera_id).mask=uint16(mask);
                images(camera_id).scores=single(scores);
                images(camera_id).tiePoints_IDs=[];
                images(camera_id).tiePoints_pix=[];
                images(camera_id).camMatrix=[];
            end
        end
    end
    close(f_waitbar)
    disp('Getting camera_matrices');
    initial_camera_id(j+1)=camera_id+1;
    images(initial_camera_id(j):camera_id)=get_camera_matrices(cameras_file{j},images(initial_camera_id(j):camera_id));
    images(initial_camera_id(j):initial_camera_id(j)-1)=[];
end

%% Point cloud reading
ptCloud.xyz=[];
for j=1:size(ptcloud_file,2)
    disp(strcat("Reading point cloud file: ", ptcloud_file{j}));
    fileID=fopen(ptcloud_file{j},'r');
    ptCloud.xyz=[ptCloud.xyz ; fscanf(fileID,'%f %f %f %d %d %d',[6 Inf])'];
    fclose(fileID);    
end
ptCloud.class=zeros(size(ptCloud.xyz(:,1)));
ptCloud.score=zeros(size(ptCloud.xyz(:,1)));
ptCloud.numDetections=zeros(size(ptCloud.xyz(:,1)));
ptCloud_det_idx=0;

%% dets projection
disp('Projecting detections from images');
f_waitbar = waitbar(0,"Projecting detections from images"); 
for i=1:size(images,2)
    %disp(strcat('Projecting detections from image: ',num2str(i)));
    waitbar(i/size(images,2),f_waitbar,strcat("Projecting image ", num2str(i), " out of ", num2str(size(images,2))));
    uv=(images(i).camMatrix'*[ptCloud.xyz(:,1:3),ones(size(ptCloud.xyz,1),1)]')';
    u=uv(:,1)./uv(:,3);
    v=uv(:,2)./uv(:,3);
    [H,W]=size(images(i).mask);
    valid_points= ((uint16(v)<H) .* (uint16(v)>0) .* (uint16(u)<W) .* (uint16(u)>0)).*(1:size(ptCloud.xyz,1))';
    valid_points= valid_points(valid_points>0);
    clear local_dets
    for det_idx=1:max(max(images(i).mask))
        local_dets(det_idx).points_ID=[];
    end

    for j=1:size(valid_points,1)
        point_idx=valid_points(j);
        if images(i).mask(uint16(v(point_idx)),uint16(u(point_idx)))>0
            det_idx=images(i).mask(uint16(v(valid_points(j))),uint16(u(valid_points(j))));
            local_dets(det_idx).points_ID=[local_dets(det_idx).points_ID; point_idx];

        end
    end


    for det_idx=1:max(max(images(i).mask))
        if images(i).scores(det_idx)<0.95
            continue
        end
        point_dets=ptCloud.xyz(local_dets(det_idx).points_ID,1:3);
        if size(point_dets,1)
            [class,type]=dbscan(point_dets,k,Eps);
            %identify if more than one clusters are projected
            if max(class)>1
                dist_dets=uv(local_dets(det_idx).points_ID,3);
                valid_class=1;
                min_dist=max(dist_dets);
                for j=1:max(class)
                    mean_dist_class=mean(dist_dets(class==j));
                    if mean_dist_class<min_dist
                        min_dist=mean_dist_class;
                        nearest_class=j;
                    end
                end
                local_dets(det_idx).points_ID=local_dets(det_idx).points_ID(class==nearest_class);
            end

            dets_overlap_idx=unique(ptCloud.class(local_dets(det_idx).points_ID));

            if size(dets_overlap_idx,1)==1 && dets_overlap_idx==0
                ptCloud_det_idx=ptCloud_det_idx+1;
                old_scores=ptCloud.score(local_dets(det_idx).points_ID);
                num_of_detections=ptCloud.numDetections(local_dets(det_idx).points_ID);
                ptCloud.class(local_dets(det_idx).points_ID)=ptCloud_det_idx;
                ptCloud.score(local_dets(det_idx).points_ID)=(old_scores.*num_of_detections + images(i).scores(det_idx))./(num_of_detections+1);
                ptCloud.numDetections(local_dets(det_idx).points_ID)=num_of_detections+1;
            else
                %find and unifying classes with overlaps
                clusters_unified=0;
                for j=1:size(dets_overlap_idx,1)
                    if dets_overlap_idx(j)==0
                        continue
                    end

                    intersection=sum(ptCloud.class(local_dets(det_idx).points_ID)==dets_overlap_idx(j));
                    size_detection=size(local_dets(det_idx).points_ID,1);
                    size_class=sum(ptCloud.class==dets_overlap_idx(j));

                    if (intersection/size_detection > IoD || intersection/size_class > IoD) && ~clusters_unified
                        clusters_unified=dets_overlap_idx(j);
                        ptCloud.class(local_dets(det_idx).points_ID)=dets_overlap_idx(j);
                        old_scores=ptCloud.score(local_dets(det_idx).points_ID);
                        num_of_detections=ptCloud.numDetections(local_dets(det_idx).points_ID);
                        ptCloud.score(local_dets(det_idx).points_ID)=(old_scores.*num_of_detections + images(i).scores(det_idx))./(num_of_detections+1);
                        ptCloud.numDetections(local_dets(det_idx).points_ID)=num_of_detections+1;
                    elseif (intersection/size_detection > IoD || intersection/size_class > IoD)
                        ptCloud.class(ptCloud.class==dets_overlap_idx(j))=clusters_unified;
                        ptCloud.class(ptCloud.class>dets_overlap_idx(j))=ptCloud.class(ptCloud.class>dets_overlap_idx(j))-1;      
                        ptCloud_det_idx=ptCloud_det_idx-1;
                    elseif ~clusters_unified && j==size(dets_overlap_idx,1)
                        ptCloud_det_idx=ptCloud_det_idx+1;
                        old_scores=ptCloud.score(local_dets(det_idx).points_ID);
                        num_of_detections=ptCloud.numDetections(local_dets(det_idx).points_ID);
                        ptCloud.class(local_dets(det_idx).points_ID)=ptCloud_det_idx;
                        ptCloud.score(local_dets(det_idx).points_ID)=(old_scores.*num_of_detections + images(i).scores(det_idx))./(num_of_detections+1);
                        ptCloud.numDetections(local_dets(det_idx).points_ID)=num_of_detections+1;
                    end         
                    %in case that we have unified two different classes
                    %we delete the unified class:
                    if ~sum(ptCloud.class==dets_overlap_idx(j))
                        ptCloud.class(ptCloud.class>dets_overlap_idx(j))=ptCloud.class(ptCloud.class>dets_overlap_idx(j))-1;      
                        ptCloud_det_idx=ptCloud_det_idx-1;
                    end

                end
            end
        end

    end    
end
close(f_waitbar)

pcDets=[ptCloud.xyz(ptCloud.class>0,:),...
     ptCloud.class(ptCloud.class>0,:),...
     ptCloud.score(ptCloud.class>0,:),...
     ptCloud.numDetections(ptCloud.class>0,:)];  
disp("Posprocessing detection ID numbers and filtering detections with a small number of points");
num_of_dets = max(pcDets(:,7));
f_waitbar = waitbar(0,"Assigning a detection ID number and filtering detections with a small number of points"); 
for i=1:max(pcDets(:,7))
   waitbar(i/num_of_dets,f_waitbar,strcat("Postprocessing detection number ", num2str(i), " out of ", num2str(num_of_dets)));
   if ~sum(pcDets(:,7)==i)
        while ~sum(pcDets(:,7)==i) && i<max(pcDets(:,7))+1
            pcDets(pcDets(:,7)>i,7)=pcDets(pcDets(:,7)>i,7)-1;
        end
   elseif sum(pcDets(:,7)==i)<10
        while sum(pcDets(:,7)==i)<10 && i<max(pcDets(:,7))+1
            pcDets(pcDets(:,7)==i,:)=[];
            pcDets(pcDets(:,7)>i,7)=pcDets(pcDets(:,7)>i,7)-1;
        end
   end
end
close(f_waitbar)



%% Colouring detections
N=max(pcDets(:,7));
map_hsv=zeros(N,3);
map_i=1:N;
map_hsv(:,1)=map_i(randperm(length(map_i)));
color_dets=zeros(size(pcDets,1),1);
for i=1:size(pcDets,1)
    pcDets(i,10)=map_hsv(uint16(pcDets(i,7)),1);
end



%% Feature extraction
disp('Feature extraction...')
CCfeatures=CCfeatureExtraction(pcDets);


%% Cluster splitting

disp('CCwmtoa split...')
CCwmtoa_K=(CCfeatures(:,2)>4000).*2;
pcDets(:,7)=CC_split(CCwmtoa_K,pcDets);
t=toc;
CCfeatures_notSplit=CCfeatures;
disp('Feature extraction after splitting...')
CCfeatures=CCfeatureExtraction(pcDets);


%% Remove FP

disp('Inicio Remove FP...')
if technique_FP
    test_modelFP=fullfile('Trained_models',strcat('SVMModelFP','_S',num2str(StandardizeFP),'-',KernelFunctionFP,'-BC',num2str(BoxConstrainFP),'.mat'));
    [pcDets,CCfeatures]=FP_removal(test_modelFP,CCfeatures,pcDets,technique_FP);
else
    for i=1:size(CCfeatures,1)  
        if CCfeatures(i,2)<10
                CCfeatures(CCfeatures(:,18)==i,1)=min([-CCfeatures(CCfeatures(:,18)==i,1),-0.5]);
                pcDets(pcDets(:,7)==i,7)=0;
        end
    end
end

for i=1:size(CCfeatures,1)
    pcDets(pcDets(:,7)==i,11)=CCfeatures(i,1);
end
pcDets(pcDets(:,7)==0,:)=[];

%% assigning color o classes 

disp("Colouring detections...")
alpha=0.5;


for i=1:max(pcDets(:,7))
   if ~sum(pcDets(:,7)==i)
        while ~sum(pcDets(:,7)==i) && i<max(pcDets(:,7))+1
            pcDets(pcDets(:,7)>i,7)=pcDets(pcDets(:,7)>i,7)-1;
        end
   end
end

N=max(pcDets(:,7));
map_hsv=zeros(N,3);
map_hsv(:,1)=1:N;
map_hsv(:,1)=map_hsv(:,1)./N;
map_hsv(:,2:3)=1;
map=hsv2rgb(map_hsv);
[~,mapidx] = sort(rand(size(map)));
map = map(bsxfun(@plus,mapidx,0:size(map,1):(numel(map)-1))); %keep columns aligned
map=[0 0 0;map];
map=(map.*255);

new_colors=pcDets(:,4:6)*(1-alpha)+map(pcDets(:,7)+1,:)*alpha;

output_pc=[pcDets(:,1:3),round(new_colors),pcDets(:,7)];



%% Save results
disp('Saving results...')
if ~exist(results_folder, 'dir')
   mkdir(results_folder)
end
results_file=fullfile(results_folder,strcat('Apple_detections.txt')); 
writematrix(output_pc,results_file,'delimiter','\t')

disp(strcat('Finished with exit in:__', num2str(toc-t), ' seg.'))
t=toc;

