TYPE HeadMeasureCommand :
STRUCT
    Start       : BOOL;
    Reset       : BOOL;
    RecipeId    : DINT;
    ConfigPath  : STRING[260];
END_STRUCT
END_TYPE

TYPE HeadMeasureResult :
STRUCT
    Done                    : BOOL;
    Busy                    : BOOL;
    Error                   : BOOL;
    ErrorCode               : DINT;
    InnerDiameterMm         : LREAL;
    InnerCircumferenceMm    : LREAL;
    RoundnessToleranceMm    : LREAL;
    StraightSideSlopeDeg    : LREAL;
    StraightSideHeightMm    : LREAL;
    CylinderFitRmsMm        : LREAL;
    CylinderFitMaxAbsMm     : LREAL;
    IcpFitRmsMm             : LREAL;
    IcpFitMaxAbsMm          : LREAL;
END_STRUCT
END_TYPE

FUNCTION_BLOCK FB_HeadMeasure
VAR_INPUT
    Cmd : HeadMeasureCommand;
END_VAR
VAR_OUTPUT
    Result : HeadMeasureResult;
END_VAR
VAR
    RisingStart : R_TRIG;
END_VAR

RisingStart(CLK := Cmd.Start);

IF Cmd.Reset THEN
    Result.Done := FALSE;
    Result.Busy := FALSE;
    Result.Error := FALSE;
    Result.ErrorCode := 0;
END_IF;

IF RisingStart.Q AND NOT Result.Busy THEN
    Result.Done := FALSE;
    Result.Error := FALSE;
    Result.ErrorCode := 0;
    Result.Busy := TRUE;

    // 实际项目中这里触发上位机/工控机测量服务：
    // 1. 发送 RecipeId 和 ConfigPath
    // 2. 等待 C++ 程序返回 JSON/DB/OPC UA 结果
    // 3. 将结果写入 Result
END_IF;

// 外部通讯完成后应置 Result.Busy := FALSE, Result.Done := TRUE。
END_FUNCTION_BLOCK
