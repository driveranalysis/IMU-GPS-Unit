% 
% pt = [0 0 0];
% dir = [1 0 0 1];
% h = quiver3(pt(1),pt(2),pt(3), dir(1),dir(2),dir(3));
% xlim([-1 1])
% ylim([-1 1])
% zlim([-1 1])
% 
% xfm = makehgtform('xrotate',0,'yrotate',0,'zrotate',0);
% newdir = xfm * dir';
% h.UData = newdir(1);
% h.VData = newdir(2);
% h.WData = newdir(3);
% 
% for i= 2:length(eul)
%       xfm = makehgtform('xrotate',eul(i,1),'yrotate',eul(i,2),'zrotate',eul(i,3));
%       newdir = xfm * dir';
%       h.UData = newdir(1);
%       h.VData = newdir(2);
%       h.WData = newdir(3);
%       drawnow
%       pause(1)
% end


%%
points = [2 1 1; 4 1 1; 4 2 1; 2 2 1; 2 2 2; 2 1 2; 4 1 2; 4 2 2];
faces = [1 2 3 4; 5 6 7 8; 2 3 8 7; 1 4 5 6; 1 2 7 6; 4 3 8 5];
translation=repmat([1 0 0],size(vertex_matrix,1),1);
points_translated = points + translation
p1 = patch('faces',faces,...
    'vertices',points,...
    'facecolor','b',...
    'edgecolor',[1,1,1],...
    'facealpha',0.5);
p1.FaceAlpha    = 0.2;

xlim([0 5])
ylim([0 5])
zlim([-5 5])
grid on
view(3)

figure
p1 = patch('faces',faces,...
    'vertices',points,...
    'facecolor','b',...
    'edgecolor',[1,1,1],...
    'facealpha',0.5);
p1.FaceAlpha    = 0.2;

xlim([0 5])
ylim([0 5])
zlim([-5 5])
grid on
view(3)

for i= 1:length(eul(:,1))
yaw = eul(i,1);
pitch = eul(i,2);
roll = eul(i,3);

rotate(p1,[0,0,1],yaw,[3 1.5 1.5]);
rotate(p1,[1,0,0],roll,[3 1.5 1.5]);
rotate(p1,[0,1,0],pitch,[3 1.5 1.5]);
drawnow
end
%% 3

maneuver_ss = maneuver_ss_extract(GroundTruth,Maneuvers_cat,fs);
maneuver_plots(AllData1_marked,maneuver_ss,Maneuvers_cat,1);
maneuver_plots(AllData1_marked,maneuver_ss,Maneuvers_cat,2);

%% 4


 AllData_binary_event = binary_event(AllData_marked)
 
