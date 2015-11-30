map=importdata('Map.txt');
figure
hold on;
for i=1:length(map(:,1))
    for j=1:length(map(1,:))
        if(map(i,j)==1)
            plot(i-1,j-1,'Marker','.','Color','b');
        elseif(map(i,j)>2)
            plot(i-1,j-1,'Marker','*','Color','r');
        end
    end
end

links=importdata('links_goncas.txt');

for i=1:length(links(:,1))
    plot([links(i,1)*100,links(i,3)*100],[links(i,2)*100,links(i,4)*100],'Color','g');
end