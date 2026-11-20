using UnityEngine;

public static class EditorFrameRateCap
{
    private const int TargetFPS = 60;

    [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.BeforeSceneLoad)]
    private static void EnforceFrameRate()
    {
        QualitySettings.vSyncCount = 0;
        Application.targetFrameRate = TargetFPS;
    }
}
