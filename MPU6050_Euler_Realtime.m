%% MPU6050 Euler realtime plotter
% Frame JustFloat: 4 x float32 little-endian + tail [00 00 80 7F]
% CH1 Roll Raw, CH2 Pitch Raw, CH3 Roll Quaternion, CH4 Pitch Quaternion.
% Edit COM_PORT below, then run this script.

clear; clc;

COM_PORT = "COM10";       % Change to the STM32 COM port
BAUD_RATE = 115200;
WINDOW_SEC = 10;
PLOT_UPDATE_SEC = 0.05;  % Refresh display at ~20 Hz; receive continuously

try
    sp = serialport(COM_PORT, BAUD_RATE, "DataBits", 8, ...
        "Parity", "none", "StopBits", 1, "Timeout", 1);
catch ME
    error("Cannot open %s. Check the COM port and close other serial monitors.\n%s", ...
        COM_PORT, ME.message);
end
sp.ByteOrder = "little-endian";
flush(sp);
cleanupObj = onCleanup(@() safeClose(sp)); %#ok<NASGU>

fig = figure("Name","MPU6050 Euler: Raw vs Quaternion", ...
    "NumberTitle","off", "Color","w");
tl = tiledlayout(fig,2,1,"TileSpacing","compact","Padding","compact");

axR = nexttile(tl);
hRraw = animatedline(axR,"DisplayName","Roll Raw","LineWidth",1.1, ...
    "Color",[0.00 0.45 0.74]);
hRq = animatedline(axR,"DisplayName","Roll Quaternion","LineWidth",1.4, ...
    "Color",[0.85 0.33 0.10]);
grid(axR,"on"); ylabel(axR,"Roll (deg)");
ylim(axR,[-45 45]);
legend(axR,"Location","northwest");
title(axR,"Roll");
currentRollText = text(axR,0.99,0.96,"Raw: --° | Quaternion: --°", ...
    "Units","normalized","HorizontalAlignment","right", ...
    "VerticalAlignment","top","FontWeight","bold", ...
    "BackgroundColor","w","Margin",3);

axP = nexttile(tl);
hPraw = animatedline(axP,"DisplayName","Pitch Raw","LineWidth",1.1, ...
    "Color",[0.47 0.67 0.19]);
hPq = animatedline(axP,"DisplayName","Pitch Quaternion","LineWidth",1.4, ...
    "Color",[0.49 0.18 0.56]);
grid(axP,"on"); ylabel(axP,"Pitch (deg)"); xlabel(axP,"Time (s)");
ylim(axP,[-45 45]);
legend(axP,"Location","northwest");
title(axP,"Pitch");
currentPitchText = text(axP,0.99,0.96,"Raw: --° | Quaternion: --°", ...
    "Units","normalized","HorizontalAlignment","right", ...
    "VerticalAlignment","top","FontWeight","bold", ...
    "BackgroundColor","w","Margin",3);

t0 = tic;
lastDraw = tic;
lastStatus = tic;
rx = uint8([]);
validFrames = 0;
badFrames = 0;
currentValues = nan(1,4);
maxRead = 4096;

fprintf("Listening on %s at %d baud.\n", COM_PORT, BAUD_RATE);
fprintf("Expected 20-byte JustFloat frame: 4 float32 + 00 00 80 7F.\n");
fprintf("Close the plot window to stop.\n");

while isvalid(fig)
    n = sp.NumBytesAvailable;
    if n > 0
        chunk = read(sp,min(n,maxRead),"uint8");
        rx = [rx; uint8(chunk(:))]; %#ok<AGROW>
    end

    marker = uint8([0 0 128 127]);
    while numel(rx) >= 20
        idx = findMarker(rx,marker);
        if isempty(idx)
            % Keep a short tail in case the marker spans the next read.
            if numel(rx) > 19
                rx = rx(end-18:end);
            end
            break;
        end

        if idx < 17
            rx(1:idx+3) = [];
            continue;
        end

        payload = rx(idx-16:idx-1);
        values = typecast(uint8(payload(:).'),"single");
        rx(1:idx+3) = [];

        if numel(values) ~= 4 || any(~isfinite(values)) || any(abs(double(values)) > 1000)
            badFrames = badFrames + 1;
            continue;
        end

        t = toc(t0);
        validFrames = validFrames + 1;
        currentValues = double(values(:).');
        addpoints(hRraw,t,double(values(1)));
        addpoints(hPraw,t,double(values(2)));
        addpoints(hRq,t,double(values(3)));
        addpoints(hPq,t,double(values(4)));
    end

    if toc(lastDraw) >= PLOT_UPDATE_SEC
        t = toc(t0);
        xlim(axR,[max(0,t-WINDOW_SEC),max(WINDOW_SEC,t)]);
        xlim(axP,[max(0,t-WINDOW_SEC),max(WINDOW_SEC,t)]);
        if all(isfinite(currentValues))
            currentRollText.String = sprintf("Raw: %+.2f°  |  Quaternion: %+.2f°", ...
                currentValues(1),currentValues(3));
            currentPitchText.String = sprintf("Raw: %+.2f°  |  Quaternion: %+.2f°", ...
                currentValues(2),currentValues(4));
        end
        drawnow limitrate;
        lastDraw = tic;
    else
        pause(0.001);
    end

    if toc(lastStatus) >= 2
        fprintf("Valid frames: %d | Invalid frames: %d\n",validFrames,badFrames);
        lastStatus = tic;
    end
end

function idx = findMarker(buf,marker)
    idx = [];
    if numel(buf) < numel(marker)
        return;
    end
    for k = 1:(numel(buf)-numel(marker)+1)
        if isequal(buf(k:k+numel(marker)-1).',marker)
            idx = k;
            return;
        end
    end
end

function safeClose(sp)
    try
        if ~isempty(sp) && isvalid(sp)
            delete(sp);
        end
    catch
    end
end
