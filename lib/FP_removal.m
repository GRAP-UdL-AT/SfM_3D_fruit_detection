function [pcDets,CCfeatures]=FP_removal(test_modelFP,CCfeatures,pcDets,technique_FP)

    disp('Inicio FP removal...')
    t=toc;
    CCfeatures_nonFP=CCfeatures;
    T=sum(CCfeatures(:,1)==0);
    TP=0;
    FP=0;
if technique_FP==1
    FP_identification= ~( (CCfeatures(:,2)>150) .*(CCfeatures(:,12)>0.995));
    
    for i=1:size(CCfeatures,1)               
        if FP_identification(i)
                CCfeatures(CCfeatures(:,18)==i,1)=min([-CCfeatures(CCfeatures(:,18)==i,1),-0.5]);
                pcDets(pcDets(:,7)==i,7)=0;
                TP=TP+1-(CCfeatures_nonFP(i,1)>0);
                FP=FP+1-(CCfeatures_nonFP(i,1)==0);
        end
    end
end    
    
    
if technique_FP==2
    SVMModelFP=loadCompactModel(test_modelFP);
    
    for i=1:size(CCfeatures,1)  
        FPprediction=predict(SVMModelFP,CCfeatures_nonFP(i,[2,6,10,12,13,14]));
        if ~FPprediction ||CCfeatures_nonFP(i,2)<10
                CCfeatures(CCfeatures(:,18)==i,1)=min([-CCfeatures(CCfeatures(:,18)==i,1),-0.5]);
                pcDets(pcDets(:,7)==i,7)=0;
                TP=TP+1-(CCfeatures_nonFP(i,1)>0);
                FP=FP+1-(CCfeatures_nonFP(i,1)==0);
        end
    end
    
end
    disp(strcat('    realizado en:__', num2str(toc-t), ' seg. __  De_',num2str(T),'_FP se han eliminado_',num2str(TP),' correctamente i_',num2str(FP),' incorrectamente'))
    