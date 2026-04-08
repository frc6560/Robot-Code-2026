#!/bin/zsh
# pull-logs.sh - Interactive script to copy match logs from roboRIO
# Team 6560

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Configuration
TEAM_NUMBER="6560"
ROBORIO_IP="10.65.60.2"
ROBORIO_HOSTNAME="roborio-${TEAM_NUMBER}-frc.local"

echo "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo "${BLUE}   Team ${TEAM_NUMBER} - Match Log Retrieval Tool${NC}"
echo "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo ""

# Prompt for event name
echo "${YELLOW}Enter event name (e.g., 'Ventura Regional', 'Week 1', 'Practice'):${NC}"
read -r EVENT_NAME

if [[ -z "$EVENT_NAME" ]]; then
    EVENT_NAME="Unnamed-Event"
    echo "${YELLOW}No event name provided, using '${EVENT_NAME}'${NC}"
fi

# Sanitize event name for filesystem (replace spaces with dashes, remove special chars)
SAFE_EVENT_NAME=$(echo "$EVENT_NAME" | sed 's/ /-/g' | sed 's/[^a-zA-Z0-9-]//g')

# Create directory structure
DATE=$(date +%Y-%m-%d)
LOCAL_LOGS_DIR="./logs/${SAFE_EVENT_NAME}/${DATE}"
mkdir -p "$LOCAL_LOGS_DIR"

echo ""
echo "${BLUE}Attempting to connect to roboRIO...${NC}"
echo "  IP: ${ROBORIO_IP}"
echo "  Hostname: ${ROBORIO_HOSTNAME}"
echo ""

# Function to try copying logs
copy_logs() {
    local source_path=$1
    local destination=$2
    local description=$3

    echo "${BLUE}Trying ${description}...${NC}"
    if scp -q -o ConnectTimeout=5 -o StrictHostKeyChecking=no -r "lvuser@${ROBORIO_IP}:${source_path}" "$destination/" 2>/dev/null; then
        return 0
    else
        return 1
    fi
}

# Try USB drive first, then fallback to home directory
SUCCESS=false

if copy_logs "/U/logs/*.wpilog" "$LOCAL_LOGS_DIR" "USB drive (/U/logs/)"; then
    SUCCESS=true
    echo "${GREEN}✓ Successfully copied logs from USB drive${NC}"
elif copy_logs "/home/lvuser/logs/*.wpilog" "$LOCAL_LOGS_DIR" "internal storage (/home/lvuser/logs/)"; then
    SUCCESS=true
    echo "${GREEN}✓ Successfully copied logs from internal storage${NC}"
else
    echo "${RED}✗ Failed to connect to roboRIO${NC}"
    echo ""
    echo "${YELLOW}Troubleshooting tips:${NC}"
    echo "  1. Check that roboRIO is powered on"
    echo "  2. Verify you're connected to the robot network"
    echo "  3. Try pinging: ping ${ROBORIO_IP}"
    echo "  4. Check USB drive is inserted in roboRIO"
    exit 1
fi

# Count and show file information
echo ""
LOG_COUNT=$(find "$LOCAL_LOGS_DIR" -name "*.wpilog" -type f 2>/dev/null | wc -l | tr -d ' ')
TOTAL_SIZE=$(du -sh "$LOCAL_LOGS_DIR" 2>/dev/null | cut -f1)

if [ "$LOG_COUNT" -eq 0 ]; then
    echo "${YELLOW}⚠ No .wpilog files found on roboRIO${NC}"
    echo "  Location checked: $LOCAL_LOGS_DIR"
    exit 0
fi

echo "${GREEN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo "${GREEN}   Copy Complete!${NC}"
echo "${GREEN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo ""
echo "  Event:         ${EVENT_NAME}"
echo "  Files copied:  ${LOG_COUNT} log file(s)"
echo "  Total size:    ${TOTAL_SIZE}"
echo "  Location:      ${LOCAL_LOGS_DIR}"
echo ""

# List the copied files
echo "${BLUE}Files copied:${NC}"
ls -lh "$LOCAL_LOGS_DIR"/*.wpilog 2>/dev/null | awk '{printf "  %-8s  %s\n", $5, $9}' | sed 's|.*/||'

echo ""

# Ask if user wants to delete logs from roboRIO
echo "${YELLOW}Delete logs from roboRIO? (y/N):${NC}"
read -r DELETE_RESPONSE

if [[ "$DELETE_RESPONSE" =~ ^[Yy]$ ]]; then
    echo ""
    echo "${YELLOW}Deleting logs from roboRIO...${NC}"

    # Try to delete from both locations
    ssh -o ConnectTimeout=5 -o StrictHostKeyChecking=no "lvuser@${ROBORIO_IP}" "rm -f /U/logs/*.wpilog" 2>/dev/null || true
    ssh -o ConnectTimeout=5 -o StrictHostKeyChecking=no "lvuser@${ROBORIO_IP}" "rm -f /home/lvuser/logs/*.wpilog" 2>/dev/null || true

    echo "${GREEN}✓ Logs deleted from roboRIO${NC}"
else
    echo "${BLUE}Logs kept on roboRIO${NC}"
fi

echo ""
echo "${GREEN}Done! Open logs in AdvantageScope:${NC}"
echo "  open ${LOCAL_LOGS_DIR}"
echo ""
