function CCfeatures=CCfeatureExtraction(pcDets)

class=pcDets(:,7);


CCfeatures=zeros(max(class),15);

f_waitbar = waitbar(0,"Extracting features"); 
for i=1:max(class)
    waitbar(i/max(class),f_waitbar,strcat("Extracting features from detection number ", num2str(i), " out of ", num2str(max(class))));
    CCxyz=pcDets(class==i,:);
    if size(CCxyz,1)
        if i==456
        end
        CCfeatures(i,2)=size(CCxyz,1); %nº of points in the cluster
        CCfeatures(i,3:5)=max(CCxyz(:,1:3))-min(CCxyz(:,1:3)); %XYZ dimensions
        [~,CCfeatures(i,6)]=boundary(CCxyz(:,1:3),0); %Volume using the boundary points
        if size(CCxyz,1)>2
            CCfeatures(i,7:9)=svd(CCxyz(:,1:3)-mean(CCxyz(:,1:3)))/sum(svd(CCxyz(:,1:3)-mean(CCxyz(:,1:3))));    
        end
        CCfeatures(i,11)=mean(CCxyz(:,8)); %mean Score
        CCfeatures(i,12)=max(CCxyz(:,8)); %max Score
        CCfeatures(i,13)=mean(CCxyz(:,9)); %mean Number of detections
        CCfeatures(i,14)=CCfeatures(i,6)./CCfeatures(i,2); %cluster point density
        CCfeatures(i,15:17)=mean(rgb2hsv(CCxyz(:,4:6)./255));
        CCfeatures(i,18)=i;
    end
end

CCfeatures(:,10)=27*prod(CCfeatures(:,7:9),2);

close(f_waitbar)
