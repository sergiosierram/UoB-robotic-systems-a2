%% First convert imported data to struct
% Input: adaptN, adaptP, fixedN and fixedP
% Output: data as struct

data = struct;
variables = ["xp", "yp", "xc", "yc"];
for testId = 1:10
    for varx = 1:4
        aux = adaptN.(strcat(variables(varx), num2str(testId)));
        aux = aux(~isnan(aux));
        data.AN.(strcat("test", num2str(testId))).(variables(varx)) = aux;
    end
    for varx = 1:4
        aux = adaptP.(strcat(variables(varx), num2str(testId)));
        aux = aux(~isnan(aux));
        data.AP.(strcat("test", num2str(testId))).(variables(varx)) = aux;
    end
    for varx = 1:4
        aux = fixedN.(strcat(variables(varx), num2str(testId)));
        aux = aux(~isnan(aux));
        data.FN.(strcat("test", num2str(testId))).(variables(varx)) = aux;
    end
    for varx = 1:4
        aux = fixedP.(strcat(variables(varx), num2str(testId)));
        aux = aux(~isnan(aux));
        data.FP.(strcat("test", num2str(testId))).(variables(varx)) = aux;
    end
end

%% Get KTE from data
condition = ["AP", "AN", "FP", "FN"];
all_ktes = [];
for c = 1:4
    for testId = 1:10
        xp = data.(condition(c)).(strcat("test",num2str(testId))).xp;
        yp = data.(condition(c)).(strcat("test",num2str(testId))).yp;
        xc = data.(condition(c)).(strcat("test",num2str(testId))).xc;
        yc = data.(condition(c)).(strcat("test",num2str(testId))).yc;
        aux_kte = [];
        for i = 1:size(xc)
            d = sqrt( (xc(i)-xp).^2 + (yc(i)-yp).^2);
            dmin = min(min(d));
            aux_kte(end+1) = dmin;
        end
        data.(condition(c)).(strcat("test",num2str(testId))).kte = sqrt(mean(aux_kte)^2 + var(aux_kte));
        all_ktes(testId, c) = sqrt(mean(aux_kte)^2 + var(aux_kte));
    end
end

%% Graph
figure;
subplot(2,2,1); grid on;
plot(data.FP.test1.xp/1000, data.FP.test1.yp/1000, 'LineWidt', 2); hold on;
plot(data.FP.test1.xc/1000, data.FP.test1.yc/1000, 'LineWidt', 2); hold on;
ylabel("Y [m]"); legend(["Recorded", "Executed"], 'Location','northwest');
subplot(2,2,2); grid on; 
plot(data.FN.test1.xp/1000, data.FN.test1.yp/1000, 'LineWidt', 2); hold on;
plot(data.FN.test1.xc/1000, data.FN.test1.yc/1000, 'LineWidt', 2); hold on;
legend(["Recorded", "Executed"], 'Location','northwest');
subplot(2,2,3); grid on; 
plot(data.AP.test1.xp/1000, data.AP.test1.yp/1000, 'LineWidt', 2); hold on;
plot(data.AP.test1.xc/1000, data.AP.test1.yc/1000, 'LineWidt', 2); hold on;
xlabel("X [m]"); ylabel("Y [m]"); legend(["Recorded", "Executed"], 'Location','northwest');
subplot(2,2,4); grid on;
plot(data.AN.test1.xp/1000, data.AN.test1.yp/1000, 'LineWidt', 2); hold on;
plot(data.AN.test1.xc/1000, data.AN.test1.yc/1000, 'LineWidt', 2); hold on;
xlabel("X [m]"); legend(["Recorded", "Executed"], 'Location','northwest');