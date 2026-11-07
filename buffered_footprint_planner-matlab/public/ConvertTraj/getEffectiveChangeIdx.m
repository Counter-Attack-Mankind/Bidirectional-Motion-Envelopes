function [change_idx_eff, seg_st, seg_ed, seg_dir, seg_len] = getEffectiveChangeIdx(x, y, v0, min_length)
% ============================================================
% getEffectiveChangeIdx
% 作用：
%   从原始方向序列 v0 中筛选"有效换向点"。
%   对于长度小于 min_length 的短方向段，不视为真实换向。
%
% 输入：
%   x, y        : 路径点坐标（已去重后的）
%   v0          : 对应每个路径点的方向符号（>0前进，<0倒车）
%   min_length  : 最短有效段长度阈值（单位：m）
%
% 输出：
%   change_idx_eff : 有效换向点索引（基于当前 x,y,v0）
%   seg_st         : 每段起点索引
%   seg_ed         : 每段终点索引
%   seg_dir        : 每段方向（+1/-1）
%   seg_len        : 每段长度
% ============================================================

    % ---------- 1) 标准化方向符号 ----------
    dir0 = sign(v0);
    if isempty(dir0)
        change_idx_eff = [];
        seg_st = [];
        seg_ed = [];
        seg_dir = [];
        seg_len = [];
        return;
    end

    % 把 0 方向点并入前一个方向；若开头就是 0，则先置为后面第一个非零方向，否则默认前进
    if dir0(1) == 0
        idx_nz = find(dir0 ~= 0, 1, 'first');
        if isempty(idx_nz)
            dir0(:) = 1;
        else
            dir0(1:idx_nz-1) = dir0(idx_nz);
        end
    end
    for i = 2:length(dir0)
        if dir0(i) == 0
            dir0(i) = dir0(i-1);
        end
    end

    % ---------- 2) 初始按方向分段 ----------
    [seg_st, seg_ed, seg_dir] = splitByDirection(dir0);

    % ---------- 3) 计算每段长度 ----------
    seg_len = computeSegLength(x, y, seg_st, seg_ed);

    % ---------- 4) 迭代删除短段 ----------
    changed = true;
    while changed
        changed = false;
        nSeg = length(seg_st);

        if nSeg <= 1
            break;
        end

        for k = 1:nSeg
            if seg_len(k) < min_length
                changed = true;

                if nSeg == 1
                    break;
                elseif k == 1
                    % 首段太短：并入下一段
                    seg_st(2) = seg_st(1);
                    seg_st(1) = [];
                    seg_ed(1) = [];
                    seg_dir(1) = [];
                elseif k == nSeg
                    % 末段太短：并入上一段
                    seg_ed(end-1) = seg_ed(end);
                    seg_st(end) = [];
                    seg_ed(end) = [];
                    seg_dir(end) = [];
                else
                    % 中间短段：只有当前后方向相同，才能直接并掉
                    % 典型情况：F - R(short) - F  或 R - F(short) - R
                    if seg_dir(k-1) == seg_dir(k+1)
                        seg_ed(k-1) = seg_ed(k+1);

                        seg_st(k:k+1) = [];
                        seg_ed(k:k+1) = [];
                        seg_dir(k:k+1) = [];
                    else
                        % 如果前后方向不同，说明结构异常，不强删，跳过
                        continue;
                    end
                end

                % 删除后重新计算长度
                seg_len = computeSegLength(x, y, seg_st, seg_ed);
                break;
            end
        end
    end

    % ---------- 5) 输出有效换向点 ----------
    if length(seg_st) <= 1
        change_idx_eff = [];
    else
        change_idx_eff = seg_st(2:end);
    end
end


% ============================================================
% 子函数1：按方向分段
% ============================================================
function [seg_st, seg_ed, seg_dir] = splitByDirection(dir0)

    seg_st = 1;
    seg_ed = [];
    seg_dir = [];

    cur_dir = dir0(1);

    for i = 2:length(dir0)
        if dir0(i) ~= cur_dir
            seg_ed(end+1)  = i-1; %#ok<AGROW>
            seg_dir(end+1) = cur_dir; %#ok<AGROW>
            seg_st(end+1)  = i; %#ok<AGROW>
            cur_dir = dir0(i);
        end
    end

    seg_ed(end+1)  = length(dir0);
    seg_dir(end+1) = cur_dir;
end


% ============================================================
% 子函数2：计算每段几何长度
% ============================================================
function seg_len = computeSegLength(x, y, seg_st, seg_ed)

    nSeg = length(seg_st);
    seg_len = zeros(1, nSeg);

    for k = 1:nSeg
        st = seg_st(k);
        ed = seg_ed(k);

        if ed <= st
            seg_len(k) = 0;
        else
            dx = diff(x(st:ed));
            dy = diff(y(st:ed));
            seg_len(k) = sum(sqrt(dx.^2 + dy.^2));
        end
    end
end