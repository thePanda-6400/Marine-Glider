% mass is in grams dimensions are in cm
mb = 1300;% battery mass 
density = 1.38; % 1.20 g/cm3 for polycarb 1.38 g/cm3 for pvc
thickness = 0.5;
dia = 11; % outer dia
len = 1000;
v = (pi*(dia/2)^2 - pi*((dia-2*thickness)/2)^2)*len;% vol 
mm = 20; %misc mass( skeleton, sensors, electronics etc)
mh = v*density + mm + mb;% total mass 

bm = v - mb - mh
