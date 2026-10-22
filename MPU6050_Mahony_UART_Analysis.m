function MPU6050_Mahony_UART_Analysis()
% MPU6050_Mahony_UART_Analysis
% Nhan UART JustFloat: 4 x single + marker 00 00 80 7F (20 byte/frame)
% Kenh: 1 Roll Raw, 2 Pitch Raw, 3 Roll Mahony, 4 Pitch Mahony
% Ghi CSV va tinh mean, std, peak-to-peak, bias so voi goc tham chieu.
%
% Chay: MPU6050_Mahony_UART_Analysis
% Mac dinh COM5, 115200 baud. Sua phan CAU HINH neu can.

    %% Cau hinh
    COM_PORT = "COM10";
    BAUD_RATE = 115200;
    PLOT_WINDOW_SECONDS = 15;
    MAX_BUFFER_SAMPLES = 20000;
    MARKER = uint8([0 0 128 127]);
    FRAME_BYTES = 20;

    %% Thong tin phep thu
    answer = inputdlg( ...
        {'Goc tham chieu Roll (do), vi du 18, -18, 0:', ...
         'Goc tham chieu Pitch (do), vi du 18, -18, 0:', ...
         'Thoi gian ghi du lieu (giay), vi du 10:', ...
         'Ten kich ban thu (vi du Roll_Plus18):'}, ...
        'Cau hinh phep thu Mahony', [1 58], ...
        {'0','0','10','Test_01'});
    if isempty(answer), return; end

    refRoll = str2double(answer{1});
    refPitch = str2double(answer{2});
    recordSeconds = str2double(answer{3});
    scenario = string(answer{4});
    if any(isnan([refRoll refPitch recordSeconds])) || recordSeconds <= 0
        errordlg('Goc tham chieu va thoi gian ghi khong hop le.','Loi cau hinh');
        return;
    end
    if strlength(strtrim(scenario)) == 0, scenario = "Test"; end

    %% Mo cong UART
    try
        sp = serialport(COM_PORT,BAUD_RATE,'Timeout',1);
        configureCallback(sp,'off');
        flush(sp);
    catch ME
        errordlg(sprintf('Khong mo duoc %s:\n%s',COM_PORT,ME.message),'Loi UART');
        return;
    end
    cleanupObj = onCleanup(@() safeCloseSerial(sp)); %#ok<NASGU>

    %% Buffer
    tData = zeros(MAX_BUFFER_SAMPLES,1);
    data = nan(MAX_BUFFER_SAMPLES,4);
    n = 0;
    frameBuffer = uint8([]);
    t0 = tic;
    isRecording = false;
    recordStartAbs = NaN;
    recordStopAbs = NaN;
    savedFile = "";

    %% Giao dien
    fig = figure('Name','MPU6050 - Mahony UART Logger', ...
        'NumberTitle','off','Color','w','Position',[100 80 1180 760], ...
        'CloseRequestFcn',@closeFigure);

    ax1 = subplot(2,1,1,'Parent',fig); hold(ax1,'on'); grid(ax1,'on');
    h1 = plot(ax1,nan,nan,'LineWidth',1.0);
    h2 = plot(ax1,nan,nan,'LineWidth',1.0);
    ylabel(ax1,'Goc (do)'); title(ax1,'Roll: Raw va Mahony');
    legend(ax1,{'Roll Raw','Roll Mahony'},'Location','northwest');
    ylim(ax1,[-45 45]);

    ax2 = subplot(2,1,2,'Parent',fig); hold(ax2,'on'); grid(ax2,'on');
    h3 = plot(ax2,nan,nan,'LineWidth',1.0);
    h4 = plot(ax2,nan,nan,'LineWidth',1.0);
    ylabel(ax2,'Goc (do)'); xlabel(ax2,'Thoi gian (s)');
    title(ax2,'Pitch: Raw va Mahony');
    legend(ax2,{'Pitch Raw','Pitch Mahony'},'Location','northwest');
    ylim(ax2,[-45 45]);

    statusText = uicontrol(fig,'Style','text','Units','normalized', ...
        'Position',[0.02 0.925 0.66 0.045],'BackgroundColor','w', ...
        'HorizontalAlignment','left','FontSize',10, ...
        'String',sprintf('UART %s @ %d | Cho du lieu...',COM_PORT,BAUD_RATE));
    btnStart = uicontrol(fig,'Style','pushbutton','Units','normalized', ...
        'Position',[0.70 0.925 0.13 0.05],'String','Bat dau ghi', ...
        'FontSize',10,'Callback',@startRecording);
    btnStop = uicontrol(fig,'Style','pushbutton','Units','normalized', ...
        'Position',[0.84 0.925 0.14 0.05],'String','Dung va tinh', ...
        'FontSize',10,'Callback',@stopAndAnalyze);
    valueText = uicontrol(fig,'Style','text','Units','normalized', ...
        'Position',[0.02 0.005 0.96 0.045],'BackgroundColor','w', ...
        'HorizontalAlignment','left','FontSize',10, ...
        'String','Raw Roll: -- | Raw Pitch: -- | Mahony Roll: -- | Mahony Pitch: --');

    configureCallback(sp,'byte',FRAME_BYTES,@readSerial);

    %% UART callback
    function readSerial(src,~)
        if ~isvalid(fig), return; end
        try
            bytesAvailable = src.NumBytesAvailable;
            if bytesAvailable <= 0, return; end
            newBytes = read(src,bytesAvailable,'uint8');
            frameBuffer = [frameBuffer; uint8(newBytes(:))]; %#ok<AGROW>

            while numel(frameBuffer) >= FRAME_BYTES
                markerPos = findMarker(frameBuffer,MARKER);
                if isempty(markerPos)
                    if numel(frameBuffer) > 3
                        frameBuffer = frameBuffer(end-2:end);
                    end
                    break;
                end
                if markerPos < 17
                    frameBuffer(1:markerPos+3) = [];
                    continue;
                end

                payload = frameBuffer(markerPos-16:markerPos-1);
                vals = typecast(uint8(payload(:)),'single');
                frameBuffer(1:markerPos+3) = [];
                if numel(vals) ~= 4 || any(~isfinite(vals)), continue; end

                n = n + 1;
                if n > MAX_BUFFER_SAMPLES
                    keep = floor(MAX_BUFFER_SAMPLES*3/4);
                    tData(1:keep) = tData(n-keep:n-1);
                    data(1:keep,:) = data(n-keep:n-1,:);
                    n = keep + 1;
                end
                tData(n) = toc(t0);
                data(n,:) = double(vals(:)).';

                idx = (1:n).';
                visible = tData(idx) >= max(0,tData(n)-PLOT_WINDOW_SECONDS);
                tx = tData(idx(visible)); dy = data(idx(visible),:);
                set(h1,'XData',tx,'YData',dy(:,1));
                set(h2,'XData',tx,'YData',dy(:,3));
                set(h3,'XData',tx,'YData',dy(:,2));
                set(h4,'XData',tx,'YData',dy(:,4));
                xlim(ax1,[max(0,tData(n)-PLOT_WINDOW_SECONDS) max(PLOT_WINDOW_SECONDS,tData(n))]);
                xlim(ax2,[max(0,tData(n)-PLOT_WINDOW_SECONDS) max(PLOT_WINDOW_SECONDS,tData(n))]);

                set(valueText,'String',sprintf( ...
                    'Raw Roll: %7.2f | Raw Pitch: %7.2f | Mahony Roll: %7.2f | Mahony Pitch: %7.2f deg', ...
                    vals(1),vals(2),vals(3),vals(4)));

                if isRecording && toc(recordStartAbs) >= recordSeconds
                    stopAndAnalyze();
                    return;
                end
            end

            if isRecording
                set(statusText,'String',sprintf( ...
                    'DANG GHI: %.1f / %.1f s | So mau: %d | Ref Roll=%.1f deg, Pitch=%.1f deg', ...
                    toc(recordStartAbs),recordSeconds,n,refRoll,refPitch));
            else
                set(statusText,'String',sprintf( ...
                    'UART %s @ %d | Da nhan %d mau | Ref Roll=%.1f deg, Pitch=%.1f deg', ...
                    COM_PORT,BAUD_RATE,n,refRoll,refPitch));
            end
            drawnow limitrate;
        catch ME
            if isvalid(statusText)
                set(statusText,'String',['Loi doc UART: ' ME.message]);
            end
        end
    end

    function startRecording(~,~)
        if n < 1
            warndlg('Chua nhan duoc frame UART hop le. Kiem tra COM, baud va firmware.','Chua co du lieu');
            return;
        end
        isRecording = true;
        recordStartAbs = toc(t0);
        recordStopAbs = NaN;
        set(btnStart,'Enable','off');
        set(btnStop,'String','Dung va tinh','Enable','on');
        set(statusText,'String',sprintf( ...
            'DANG GHI: 0 / %.1f s | Ref Roll=%.1f deg, Pitch=%.1f deg', ...
            recordSeconds,refRoll,refPitch));
    end

    function stopAndAnalyze(~,~)
        if ~isRecording, return; end
        isRecording = false;
        recordStopAbs = toc(t0);
        set(btnStart,'Enable','on');
        set(btnStop,'String','Tinh xong','Enable','on');

        valid = find(tData(1:n) >= recordStartAbs & tData(1:n) <= recordStopAbs);
        if numel(valid) < 5
            warndlg('Qua it mau trong lan ghi. Hay thu lai voi thoi gian dai hon.','Khong du du lieu');
            return;
        end

        elapsed = tData(valid)-tData(valid(1));
        x = data(valid,:);
        T = table(elapsed,x(:,1),x(:,2),x(:,3),x(:,4), ...
            'VariableNames',{'Time_s','Roll_Raw_deg','Pitch_Raw_deg', ...
                             'Roll_Mahony_deg','Pitch_Mahony_deg'});

        stamp = datestr(now,'yyyymmdd_HHMMSS');
        safeScenario = regexprep(char(scenario),'[^a-zA-Z0-9_-]','_');
        defaultName = sprintf('%s_Mahony_%s.csv',safeScenario,stamp);
        [file,folder] = uiputfile('*.csv','Luu du lieu UART CSV',defaultName);
        savedFile = "";
        if ~isequal(file,0)
            savedFile = fullfile(folder,file);
            writetable(T,savedFile);
        end

        Algorithm = ["Raw";"Mahony"];
        rollCols = [2 4]; pitchCols = [3 5];
        MeanRoll_deg = zeros(2,1); StdRoll_deg = zeros(2,1);
        P2PRoll_deg = zeros(2,1); RollBias_deg = zeros(2,1);
        MeanPitch_deg = zeros(2,1); StdPitch_deg = zeros(2,1);
        P2PPitch_deg = zeros(2,1); PitchBias_deg = zeros(2,1);
        for ii = 1:2
            r = T{:,rollCols(ii)}; p = T{:,pitchCols(ii)};
            MeanRoll_deg(ii) = mean(r,'omitnan');
            StdRoll_deg(ii) = std(r,0,'omitnan');
            P2PRoll_deg(ii) = max(r)-min(r);
            RollBias_deg(ii) = MeanRoll_deg(ii)-refRoll;
            MeanPitch_deg(ii) = mean(p,'omitnan');
            StdPitch_deg(ii) = std(p,0,'omitnan');
            P2PPitch_deg(ii) = max(p)-min(p);
            PitchBias_deg(ii) = MeanPitch_deg(ii)-refPitch;
        end
        Stats = table(Algorithm,MeanRoll_deg,StdRoll_deg,P2PRoll_deg,RollBias_deg, ...
            MeanPitch_deg,StdPitch_deg,P2PPitch_deg,PitchBias_deg);

        fprintf('\n========== KET QUA THU MAHONY ==========\n');
        fprintf('Kich ban: %s\n',scenario);
        fprintf('Goc tham chieu danh dinh: Roll = %.2f deg, Pitch = %.2f deg\n',refRoll,refPitch);
        fprintf('So mau: %d | Thoi gian: %.2f s\n',height(T),elapsed(end));
        disp(Stats);
        if strlength(savedFile) > 0
            [folder,base,~] = fileparts(savedFile);
            statsFile = fullfile(folder,[base '_stats.csv']);
            writetable(Stats,statsFile);
            fprintf('CSV da luu: %s\nBang thong ke da luu: %s\n',savedFile,statsFile);
        else
            fprintf('Nguoi dung da huy luu CSV.\n');
        end
        set(statusText,'String',sprintf('Da tinh xong: %d mau | Ref Roll %.1f deg | Pitch %.1f deg', ...
            height(T),refRoll,refPitch));
    end

    function closeFigure(~,~)
        try, configureCallback(sp,'off'); catch, end
        delete(fig);
    end
end

function pos = findMarker(bytes,marker)
    pos = [];
    if numel(bytes) < numel(marker), return; end
    for k = 1:(numel(bytes)-numel(marker)+1)
        if all(bytes(k:k+numel(marker)-1) == marker(:))
            pos = k;
            return;
        end
    end
end

function safeCloseSerial(sp)
    try, configureCallback(sp,'off'); catch, end
    try, delete(sp); catch, end
end
