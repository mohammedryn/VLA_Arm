#!/bin/bash
# Train ACT policy for RoArm M2-S pick-and-place
# Run on GPU machine after Ryan pushes the dataset (Handoff H2).
#
# Replace DATASET_ID with the repo_id Ryan sends you.
# Replace HF_USERNAME with your HuggingFace username for the checkpoint push.

DATASET_ID="${1:-ryanm/roarm_pickplace_v1}"
HF_USERNAME="${2:-YOUR_HF_USERNAME}"

lerobot-train \
  --dataset.repo_id="$DATASET_ID" \
  --policy.type=act \
  --policy.chunk_size=100 \
  --policy.n_action_steps=100 \
  --policy.input_normalization_modes.observation.state=mean_std \
  --policy.input_normalization_modes.observation.images.wrist=mean_std \
  --policy.output_normalization_modes.action=mean_std \
  --training.num_workers=4 \
  --training.batch_size=8 \
  --training.num_steps=80000 \
  --training.eval_freq=5000 \
  --training.save_freq=10000 \
  --output_dir=outputs/roarm_act_v1 \
  --wandb.enable=true \
  --wandb.project=roarm_act

echo ""
echo "Training done. Push checkpoint to HuggingFace:"
echo "  huggingface-cli upload ${HF_USERNAME}/roarm_act_v1 \\"
echo "    outputs/roarm_act_v1/checkpoints/last/pretrained_model ."
echo ""
echo "Then tell Ryan:"
echo "  Checkpoint: ${HF_USERNAME}/roarm_act_v1"
echo "  Run: python scripts/rollout_roarm.py --model_id ${HF_USERNAME}/roarm_act_v1"
