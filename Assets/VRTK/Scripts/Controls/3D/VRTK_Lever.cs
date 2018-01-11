// Lever|Controls3D|100070
namespace VRTK
{
	using System;
	using UnityEngine;

    /// <summary>
    /// Attaching the script to a game object will allow the user to interact with it as if it were a lever. The direction can be freely set.
    /// </summary>
    /// <remarks>
    /// The script will instantiate the required Rigidbody, Interactable and HingeJoint components automatically in case they do not exist yet. The joint is very tricky to setup automatically though and will only work in straight forward cases. If there are any issues, then create the HingeJoint component manually and configure it as needed.
    /// </remarks>
    /// <example>
    /// `VRTK/Examples/025_Controls_Overview` has a couple of levers that can be grabbed and moved. One lever is horizontal and the other is vertical.
    /// </example>
    [AddComponentMenu("VRTK/Scripts/Controls/3D/VRTK_Lever")]
    public class VRTK_Lever : VRTK_Control
    {
        public enum LeverDirection
        {
            x,
            y,
            z
        }

        [Tooltip("An optional game object to which the lever will be connected. If the game object moves the lever will follow along.")]
        public GameObject connectedTo;
        [Tooltip("The axis on which the lever should rotate. All other axis will be frozen.")]
        public LeverDirection direction = LeverDirection.y;
        [Tooltip("The minimum angle of the lever counted from its initial position.")]
        public float minAngle = 0f;
        [Tooltip("The maximum angle of the lever counted from its initial position.")]
        public float maxAngle = 130f;
        [Tooltip("The increments in which lever values can change.")]
        public float stepSize = 1f;
        [Tooltip("The amount of friction the lever will have whilst swinging when it is not grabbed.")]
        public float releasedFriction = 30f;
        [Tooltip("The amount of friction the lever will have whilst swinging when it is grabbed.")]
        public float grabbedFriction = 60f;

        protected HingeJoint leverHingeJoint;
        protected bool leverHingeJointCreated = false;
        protected Rigidbody leverRigidbody;
		protected Quaternion enableAngle;	// The angle of the attached body when it was first activated
		protected Quaternion startAngle;	// The angle of the attached body when it was most recently enabled
		protected float hingeDisableAngle;  // The angle that the hinge joint was rotated when it was last disabled
		protected float hingeOffsetAngle;	// The offset angle of the hinge joint based on the value it had when it was last disabled and taking into account attached body angles


		protected override void Awake()
		{
			base.Awake();

			// Store initial rotation
			startAngle = transform.localRotation;
		}

		protected virtual void OnEnable()
		{
			// Store enabled rotation
			enableAngle = transform.localRotation;

			// The offset angle is the original start angle offset by the angle the handle
			// was in when it was enabled plus the amount the handle had been rotated at the time 
			// it was disabled. On first enable this offset will be zero. But 
			// if the user moves the handle, then disables and re-enables the handle, there 
			// will be offsets.

			// Calculate offset angle
			float enable = GetEnableAngle();
			float start = GetStartAngle();

			hingeOffsetAngle = (enable - start) + hingeDisableAngle;

			// Handle the fact that hinge joints reset their angles on Disable and Re-Enable.
			HandleEnableAngles();
		}

		protected virtual void OnDisable()
		{
			if (leverHingeJoint != null)
			{
				hingeDisableAngle = (leverHingeJoint.angle + hingeDisableAngle);
			}
		}
		
		/// <summary>
		/// Gets Euler from the specified quaternion that matches the current handle direction.
		/// </summary>
		/// <param name="quaternion"></param>
		protected float GetHandleAngle(Quaternion quaternion)
		{
			switch (direction)
			{
				case LeverDirection.x:
					return quaternion.eulerAngles.x;
				case LeverDirection.y:
					return quaternion.eulerAngles.y;
				case LeverDirection.z:
				default:
					return quaternion.eulerAngles.z;
			}
		}

		/// <summary>
		/// Gets the angle of the handle when the control was started based on the handle direction.
		/// </summary>
		protected float GetStartAngle()
		{
			return GetHandleAngle(startAngle);
		}

		/// <summary>
		/// Gets the angle of the handle when the control was enabled based on the handle direction.
		/// </summary>
		protected float GetEnableAngle()
		{
			return GetHandleAngle(enableAngle);
		}

		protected virtual void HandleEnableAngles()
		{
			/*
			 * Whenever a HingeJoint is enabled, it's starting angle is offset by the parent transform.
			 * For example:
			 * 
			 * - If the parents starting rotation angle is 0 degrees and the springs target angle is 45 degrees,
			 * the hinge will be rotated +45 degrees in world space even though its local angles are not. 
			 * 
			 * If a hinge is disable and re-enabled the rotation angle will always be off unless something is done to compensate.
			 * 
			 */
			if (leverHingeJoint != null)
			{
				float enable = GetHandleAngle(enableAngle);
				float start = GetHandleAngle(startAngle);
				// Since the joint resets its own angles relative to the parent on ever activation, 
				// We now need to calculate how to offset the joints to match where the lever was 
				// before it was deactivated.
				JointLimits leverJointLimits = leverHingeJoint.limits;
				leverJointLimits.min = (minAngle - (enable + hingeDisableAngle));
				leverJointLimits.max = (maxAngle - (enable + hingeDisableAngle));
				leverHingeJoint.limits = leverJointLimits;
			}
		}

		protected override void InitRequiredComponents()
        {
            if (GetComponentInChildren<Collider>() == null)
            {
                VRTK_SharedMethods.CreateColliders(gameObject);
            }

            InitRigidbody();
            InitInteractableObject();
            InitHingeJoint();
        }

        protected override bool DetectSetup()
        {
            if (leverHingeJointCreated)
            {
                Bounds bounds = VRTK_SharedMethods.GetBounds(transform, transform);
                switch (direction)
                {
                    case LeverDirection.x:
                        leverHingeJoint.anchor = (bounds.extents.y > bounds.extents.z) ? new Vector3(0, bounds.extents.y / transform.lossyScale.y, 0) : new Vector3(0, 0, bounds.extents.z / transform.lossyScale.z);
                        break;
                    case LeverDirection.y:
                        leverHingeJoint.axis = new Vector3(0, 1, 0);
                        leverHingeJoint.anchor = (bounds.extents.x > bounds.extents.z) ? new Vector3(bounds.extents.x / transform.lossyScale.x, 0, 0) : new Vector3(0, 0, bounds.extents.z / transform.lossyScale.z);
                        break;
                    case LeverDirection.z:
                        leverHingeJoint.axis = new Vector3(0, 0, 1);
                        leverHingeJoint.anchor = (bounds.extents.y > bounds.extents.x) ? new Vector3(0, bounds.extents.y / transform.lossyScale.y, 0) : new Vector3(bounds.extents.x / transform.lossyScale.x, 0);
                        break;
                }
                leverHingeJoint.anchor *= -1; // subdirection detection not yet implemented
            }

            if (leverHingeJoint)
            {
                leverHingeJoint.useLimits = true;
                JointLimits leverJointLimits = leverHingeJoint.limits;
                leverJointLimits.min = minAngle;
                leverJointLimits.max = maxAngle;
                leverHingeJoint.limits = leverJointLimits;

                if (connectedTo)
                {
                    leverHingeJoint.connectedBody = connectedTo.GetComponent<Rigidbody>();
                }
            }

            return true;
        }

        protected override ControlValueRange RegisterValueRange()
        {
            return new ControlValueRange()
            {
                controlMin = minAngle,
                controlMax = maxAngle
            };
        }

        protected override void HandleUpdate()
        {
            value = CalculateValue();
            SnapToValue(value);
        }

        protected virtual void InitRigidbody()
        {
            leverRigidbody = GetComponent<Rigidbody>();
            if (leverRigidbody == null)
            {
                leverRigidbody = gameObject.AddComponent<Rigidbody>();
                leverRigidbody.collisionDetectionMode = CollisionDetectionMode.ContinuousDynamic;
                leverRigidbody.angularDrag = releasedFriction; // otherwise lever will continue to move too far on its own
            }
            leverRigidbody.isKinematic = false;
            leverRigidbody.useGravity = false;
        }

        protected virtual void InitInteractableObject()
        {
            VRTK_InteractableObject leverInteractableObject = GetComponent<VRTK_InteractableObject>();
            if (leverInteractableObject == null)
            {
                leverInteractableObject = gameObject.AddComponent<VRTK_InteractableObject>();
            }
            leverInteractableObject.isGrabbable = true;
            leverInteractableObject.grabAttachMechanicScript = gameObject.AddComponent<GrabAttachMechanics.VRTK_RotatorTrackGrabAttach>();
            leverInteractableObject.secondaryGrabActionScript = gameObject.AddComponent<SecondaryControllerGrabActions.VRTK_SwapControllerGrabAction>();
            leverInteractableObject.grabAttachMechanicScript.precisionGrab = true;
            leverInteractableObject.stayGrabbedOnTeleport = false;

            leverInteractableObject.InteractableObjectGrabbed += InteractableObjectGrabbed;
            leverInteractableObject.InteractableObjectUngrabbed += InteractableObjectUngrabbed;
        }

        protected virtual void InteractableObjectGrabbed(object sender, InteractableObjectEventArgs e)
        {
            leverRigidbody.angularDrag = grabbedFriction;
        }

        protected virtual void InteractableObjectUngrabbed(object sender, InteractableObjectEventArgs e)
        {
            leverRigidbody.angularDrag = releasedFriction;
        }

        protected virtual void InitHingeJoint()
        {
            leverHingeJoint = GetComponent<HingeJoint>();
            if (leverHingeJoint == null)
            {
                leverHingeJoint = gameObject.AddComponent<HingeJoint>();
                leverHingeJointCreated = true;
            }

            if (connectedTo)
            {
                Rigidbody leverConnectedToRigidbody = connectedTo.GetComponent<Rigidbody>();
                if (leverConnectedToRigidbody == null)
                {
                    leverConnectedToRigidbody = connectedTo.AddComponent<Rigidbody>();
                }
                leverConnectedToRigidbody.useGravity = false;
            }
        }

        protected virtual float CalculateValue()
        {
			var enable = GetEnableAngle();
            return Mathf.Round(((leverHingeJoint.angle + enable) + hingeDisableAngle) / stepSize) * stepSize;
        }

        protected virtual void SnapToValue(float value)
        {
			float angle = ((value - minAngle) / (maxAngle - minAngle)) * (leverHingeJoint.limits.max - leverHingeJoint.limits.min);

			// TODO: there is no direct setter, one recommendation by Unity staff is to "abuse" min/max which seems the most reliable but not working so far
			JointLimits oldLimits = leverHingeJoint.limits;
			JointLimits tempLimits = leverHingeJoint.limits;
			tempLimits.min = angle;
			tempLimits.max = angle;
			leverHingeJoint.limits = tempLimits;
			leverHingeJoint.limits = oldLimits;
		}
    }
}