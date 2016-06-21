% Fonction permettant de visualiser les descripteurs en 3D
%    - displayFeatures3d(feat)
%    - displayFeatures3d(feat,group) si on a les classes d'appartenance de
%    chaque pixel
%
function displayFeatures3d(feat,group)

% controle des entrees
if nargin < 2
    group = ones(size(feat,1),1);
end
ncl = numel(unique(group));

% options d'affichage
clr = 'bgrcmyk'; % color for each group 'bgrcmyk'
sym = 'o+*oc+*';

% affichage
hold on
for i = 1 : ncl
    sub = find(group == i);
    scatter3(feat(sub,1), feat(sub,2), feat(sub,3), [], clr(i), sym(i))
end
hold off

% options d'affichage
xlabel('Rouge'); 
ylabel('Vert'); 
zlabel('Bleu')


end