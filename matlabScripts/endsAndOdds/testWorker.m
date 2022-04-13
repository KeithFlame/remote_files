N=10e6;
tic;
ticBytes(gcp);
parfor i=1:N
    a=norm(ones(10,10))*1.343+1.222;
end
toc;
tocBytes(gcp);
tic;
for i=1:N
    a=norm(ones(10,10))*1.343+1.222;
end
toc;