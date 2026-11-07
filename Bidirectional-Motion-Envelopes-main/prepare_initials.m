function prepare_initials()
% Fresh Hybrid A* searches; no published trajectories are reused.
here=fileparts(mfilename('fullpath'));
addpath(fullfile(here,'source','CommonParking'));SetupCommonParking();
out=fullfile(here,'inputs');if ~isfolder(out),mkdir(out);end
cfg=BenchmarkConfig(); options=hacg.Config();
for id=1:12
    c=LoadCase(id);timer=tic;
    geometry=hacg.Geometry(c);
    raw=parking.SearchHybridAStar(c,options.search,geometry);
    elapsed=toc(timer);
    s=struct('case_id',id,'case_data',c,'search',raw,'search_wall_s',elapsed, ...
        'benchmark_commit','476ef8573d9e22578be5c3d42514a9a29f27d07a');
    if raw.success
        path=cp.PathFromArcs([c.task.x0,c.task.y0,c.task.theta0],raw.primitives,c.vehicle,0.02);
        ref=cpe.MakeReference(struct('kind','path','path',path),c);
        % Include every gear reversal and timing phase boundary exactly.
        tt=0;
        for j=1:numel(ref.profiles)
            pr=ref.profiles{j};cuts=unique([0,pr.ta,pr.ta+pr.tc,pr.tf]);
            for k=1:numel(cuts)-1
                local=linspace(cuts(k),cuts(k+1),max(1,ceil((cuts(k+1)-cuts(k))/0.24))+1);
                tt=[tt;ref.times(j)+local(2:end)']; %#ok<AGROW>
            end
        end
        z=cpe.ReferenceAt(ref,tt,c.vehicle);
        z.omega=[diff(z.phi)./diff(tt);0];
        z.omega=max(-c.vehicle.wmax,min(c.vehicle.wmax,z.omega));
        z.phi([1,end])=0;z.v([1,end])=0;z.a([1,end])=0;z.omega([1,end])=0;
        s.initial=z;s.path=path;s.reference=ref;
        fprintf('CASE %02d OK: %.3f s search, %.3f m, %.3f s timing, %d nodes\n', ...
            id,elapsed,raw.path_length,ref.tf,numel(tt));
    else
        fprintf('CASE %02d SEARCH FAILED after %.3f s\n',id,elapsed);
    end
    f=fopen(fullfile(out,sprintf('case%02d.json',id)),'w');
    fwrite(f,jsonencode(s),'char');fclose(f);
    save(fullfile(out,sprintf('case%02d.mat',id)),'s','-v7');
end
end
