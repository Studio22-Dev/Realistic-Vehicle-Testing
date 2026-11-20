using UnityEngine;
#if ENABLE_INPUT_SYSTEM
using UnityEngine.InputSystem;
#endif

public class VehicleFollowCamera : MonoBehaviour
{
    public enum DragButton
    {
        Left = 0,
        Right = 1,
        Middle = 2
    }

    [Header("References")]
    [Tooltip("The camera transform to move. If left empty, Camera.main will be used.")]
    [SerializeField] private Transform cameraTransform;

    [Header("Default Framing")]
    [Tooltip("Distance behind the vehicle.")]
    [SerializeField] private float followDistance = 7f;

    [Tooltip("Height offset above the vehicle's pivot.")]
    [SerializeField] private float heightOffset = 2f;

    [Tooltip("Slight downward tilt so the vehicle sits cleanly in the lower-middle of the screen.")]
    [SerializeField] private float lookDownAngle = 8f;

    [Header("Gran Turismo / Forza Loose Follow")]
    [Tooltip("How loosely the camera follows steering/turning (Y-axis). Higher = looser.")]
    [SerializeField] private float yawSmoothTime = 0.18f;

    [Tooltip("How loosely the camera follows hills/bumps (X-axis). Higher = looser.")]
    [SerializeField] private float pitchSmoothTime = 0.25f;

    [Header("Click & Drag Orbit")]
    [Tooltip("Mouse button used to orbit the camera around the vehicle.")]
    [SerializeField] private DragButton dragMouseButton = DragButton.Left;

    [Tooltip("Mouse sensitivity when orbiting.")]
    [SerializeField] private float orbitSensitivity = 3.5f;

    [Tooltip("Lowest pitch angle offset (in degrees) when orbiting.")]
    [SerializeField] private float minPitchOffset = -20f;

    [Tooltip("Highest pitch angle offset (in degrees) when orbiting.")]
    [SerializeField] private float maxPitchOffset = 65f;

    [Tooltip("Inverts vertical drag direction.")]
    [SerializeField] private bool invertY = false;

    [Header("Auto-Return Timer")]
    [Tooltip("Seconds to hold the orbited angle after releasing the mouse before returning.")]
    [SerializeField] private float returnDelay = 2f;

    [Tooltip("How quickly the camera lerps back to default framing once the timer finishes.")]
    [SerializeField] private float returnLerpSpeed = 5f;

    [Header("Hierarchy")]
    [Tooltip("Unparents the camera at Start if it is a child of the vehicle.")]
    [SerializeField] private bool detachCameraOnStart = true;

    private float _currentYaw;
    private float _currentPitch;
    private float _yawVelocity;
    private float _pitchVelocity;

    private float _orbitYawOffset;
    private float _orbitPitchOffset;
    private float _timeSinceLastDrag;
    private bool _wasDragging;

    private void Start()
    {
        if (cameraTransform == null)
        {
            if (Camera.main != null)
            {
                cameraTransform = Camera.main.transform;
            }
            else
            {
                Debug.LogWarning($"{nameof(VehicleFollowCamera)}: No camera assigned and no MainCamera found in the scene.", this);
                return;
            }
        }

        if (detachCameraOnStart && cameraTransform.IsChildOf(transform))
        {
            cameraTransform.SetParent(null);
        }

        // Start with the return timer already satisfied and angles matched to the vehicle
        _timeSinceLastDrag = returnDelay;
        GetTargetAngles(out _currentYaw, out _currentPitch);
        ApplyCameraTransform();
    }

    private void LateUpdate()
    {
        if (cameraTransform == null) return;

        // 1. Update loose vehicle follow angles (X and Y)
        GetTargetAngles(out float targetYaw, out float targetPitch);
        _currentYaw = Mathf.SmoothDampAngle(_currentYaw, targetYaw, ref _yawVelocity, yawSmoothTime);
        _currentPitch = Mathf.SmoothDampAngle(_currentPitch, targetPitch, ref _pitchVelocity, pitchSmoothTime);

        // 2. Handle click-and-drag orbiting & auto-return timer
        HandleOrbitInput();

        // 3. Apply combined framing with Z locked at 0
        ApplyCameraTransform();
    }

    private void HandleOrbitInput()
    {
        GetMouseDragInput(out bool isButtonPressed, out Vector2 mouseDelta);

        if (isButtonPressed)
        {
            // Ignore delta on the very first frame the button is pressed to prevent snapping
            if (_wasDragging)
            {
                _orbitYawOffset += mouseDelta.x * orbitSensitivity;

                float ySign = invertY ? 1f : -1f;
                _orbitPitchOffset = Mathf.Clamp(
                    _orbitPitchOffset + mouseDelta.y * orbitSensitivity * ySign,
                    minPitchOffset,
                    maxPitchOffset
                );
            }

            _wasDragging = true;
            _timeSinceLastDrag = 0f;
        }
        else
        {
            _wasDragging = false;
            _timeSinceLastDrag += Time.deltaTime;

            // Once the timer reaches returnDelay, smoothly lerp back to default framing
            if (_timeSinceLastDrag >= returnDelay)
            {
                float lerpFactor = 1f - Mathf.Exp(-returnLerpSpeed * Time.deltaTime);

                // LerpAngle ensures yaw always takes the shortest path back to 0
                _orbitYawOffset = Mathf.LerpAngle(_orbitYawOffset, 0f, lerpFactor);
                _orbitPitchOffset = Mathf.Lerp(_orbitPitchOffset, 0f, lerpFactor);

                if (Mathf.Abs(_orbitYawOffset) < 0.01f) _orbitYawOffset = 0f;
                if (Mathf.Abs(_orbitPitchOffset) < 0.01f) _orbitPitchOffset = 0f;
            }
        }
    }

    private void GetMouseDragInput(out bool isPressed, out Vector2 delta)
    {
#if ENABLE_INPUT_SYSTEM
        if (Mouse.current == null)
        {
            isPressed = false;
            delta = Vector2.zero;
            return;
        }

        isPressed = dragMouseButton switch
        {
            DragButton.Left => Mouse.current.leftButton.isPressed,
            DragButton.Right => Mouse.current.rightButton.isPressed,
            DragButton.Middle => Mouse.current.middleButton.isPressed,
            _ => false
        };

        // Scale pixel delta down so sensitivity matches Legacy Input Manager values
        delta = Mouse.current.delta.ReadValue() * 0.05f;
#else
        isPressed = Input.GetMouseButton((int)dragMouseButton);
        delta = new Vector2(Input.GetAxis("Mouse X"), Input.GetAxis("Mouse Y"));
#endif
    }

    private void GetTargetAngles(out float targetYaw, out float targetPitch)
    {
        Vector3 forward = transform.forward;

        Vector3 flatForward = new Vector3(forward.x, 0f, forward.z);
        if (flatForward.sqrMagnitude > 0.001f)
        {
            targetYaw = Mathf.Atan2(flatForward.x, flatForward.z) * Mathf.Rad2Deg;
        }
        else
        {
            targetYaw = _currentYaw;
        }

        targetPitch = -Mathf.Asin(Mathf.Clamp(forward.y, -1f, 1f)) * Mathf.Rad2Deg;
    }

    private void ApplyCameraTransform()
    {
        float finalYaw = _currentYaw + _orbitYawOffset;

        // Clamp total pitch so (finalPitch + lookDownAngle) never exceeds +/- 85 deg and flips Z
        float finalPitch = Mathf.Clamp(
            _currentPitch + _orbitPitchOffset,
            -85f - lookDownAngle,
            85f - lookDownAngle
        );

        Quaternion orbitRotation = Quaternion.Euler(finalPitch, finalYaw, 0f);

        Vector3 targetPosition = transform.position
            - orbitRotation * Vector3.forward * followDistance
            + orbitRotation * Vector3.up * heightOffset;

        cameraTransform.SetPositionAndRotation(targetPosition, Quaternion.Euler(finalPitch + lookDownAngle, finalYaw, 0f));
    }
}
