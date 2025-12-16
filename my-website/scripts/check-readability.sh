#!/bin/bash
#
# check-readability.sh
#
# Purpose: Check Flesch-Kincaid readability (target: grade 9-12)
# Usage: ./scripts/check-readability.sh [file-or-directory]
#
# This script analyzes markdown files to ensure they meet the
# Flesch-Kincaid grade level 9-12 requirement (readability score 60-70).
#

set -euo pipefail

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Configuration
TARGET="${1:-docs/module-01-ros2}"
MIN_SCORE=60
MAX_SCORE=70

echo -e "${YELLOW}================================${NC}"
echo -e "${YELLOW}Readability Check${NC}"
echo -e "${YELLOW}================================${NC}"
echo ""
echo "Target: $TARGET"
echo "Expected Flesch-Kincaid: $MIN_SCORE-$MAX_SCORE (Grade 9-12)"
echo ""

# Check if textstat is installed (Python library for readability metrics)
if ! python3 -c "import textstat" 2>/dev/null; then
    echo -e "${YELLOW}Installing textstat library...${NC}"
    pip3 install textstat --quiet || {
        echo -e "${RED}ERROR: Failed to install textstat${NC}"
        echo "Please install manually: pip3 install textstat"
        exit 1
    }
fi

# Find all markdown files
if [ -f "$TARGET" ]; then
    MD_FILES=("$TARGET")
elif [ -d "$TARGET" ]; then
    mapfile -t MD_FILES < <(find "$TARGET" -name "*.md" -type f ! -path "*/node_modules/*" ! -path "*/_templates/*")
else
    echo -e "${RED}ERROR: Target not found: $TARGET${NC}"
    echo "Usage: $0 [file-or-directory]"
    exit 1
fi

if [ ${#MD_FILES[@]} -eq 0 ]; then
    echo -e "${YELLOW}WARNING: No markdown files found in $TARGET${NC}"
    exit 0
fi

echo -e "${GREEN}Found ${#MD_FILES[@]} markdown file(s)${NC}"
echo ""

# Create Python script for readability analysis
PYTHON_SCRIPT=$(cat <<'EOF'
import sys
import re
import textstat

def clean_markdown(text):
    """Remove markdown syntax for accurate readability analysis"""
    # Remove code blocks
    text = re.sub(r'```[\s\S]*?```', '', text)
    # Remove inline code
    text = re.sub(r'`[^`]+`', '', text)
    # Remove links but keep text
    text = re.sub(r'\[([^\]]+)\]\([^\)]+\)', r'\1', text)
    # Remove images
    text = re.sub(r'!\[([^\]]*)\]\([^\)]+\)', '', text)
    # Remove headers
    text = re.sub(r'^#+\s+', '', text, flags=re.MULTILINE)
    # Remove bold/italic
    text = re.sub(r'\*\*([^\*]+)\*\*', r'\1', text)
    text = re.sub(r'\*([^\*]+)\*', r'\1', text)
    # Remove frontmatter
    text = re.sub(r'^---[\s\S]*?---', '', text)
    # Remove HTML tags
    text = re.sub(r'<[^>]+>', '', text)
    return text.strip()

def analyze_file(filepath):
    """Analyze readability of a markdown file"""
    try:
        with open(filepath, 'r', encoding='utf-8') as f:
            content = f.read()

        cleaned_text = clean_markdown(content)

        if len(cleaned_text.split()) < 100:
            return None, "Too short (< 100 words)"

        # Flesch Reading Ease (higher = easier)
        flesch_score = textstat.flesch_reading_ease(cleaned_text)
        # Flesch-Kincaid Grade Level
        fk_grade = textstat.flesch_kincaid_grade(cleaned_text)

        return {
            'flesch_score': flesch_score,
            'fk_grade': fk_grade,
            'word_count': len(cleaned_text.split()),
        }, None

    except Exception as e:
        return None, str(e)

if __name__ == "__main__":
    filepath = sys.argv[1]
    result, error = analyze_file(filepath)

    if error:
        print(f"ERROR:{error}")
        sys.exit(1)

    print(f"{result['flesch_score']:.1f}|{result['fk_grade']:.1f}|{result['word_count']}")
EOF
)

# Analyze each markdown file
FAILED_FILES=()
PASSED_FILES=()
SKIPPED_FILES=()

for MD_FILE in "${MD_FILES[@]}"; do
    FILE_NAME=$(basename "$MD_FILE")
    echo -n "Checking $FILE_NAME... "

    RESULT=$(python3 -c "$PYTHON_SCRIPT" "$MD_FILE" 2>&1 || echo "ERROR:Analysis failed")

    if [[ "$RESULT" == ERROR:* ]]; then
        ERROR_MSG="${RESULT#ERROR:}"
        if [[ "$ERROR_MSG" == "Too short"* ]]; then
            echo -e "${YELLOW}SKIPPED ($ERROR_MSG)${NC}"
            SKIPPED_FILES+=("$FILE_NAME: $ERROR_MSG")
        else
            echo -e "${RED}ERROR ($ERROR_MSG)${NC}"
            FAILED_FILES+=("$FILE_NAME: $ERROR_MSG")
        fi
    else
        IFS='|' read -r FLESCH_SCORE FK_GRADE WORD_COUNT <<< "$RESULT"

        # Check if within target range
        if (( $(echo "$FLESCH_SCORE >= $MIN_SCORE" | bc -l) )) && \
           (( $(echo "$FLESCH_SCORE <= $MAX_SCORE" | bc -l) )); then
            echo -e "${GREEN}PASS (Score: $FLESCH_SCORE, Grade: $FK_GRADE, Words: $WORD_COUNT)${NC}"
            PASSED_FILES+=("$FILE_NAME")
        else
            echo -e "${RED}FAIL (Score: $FLESCH_SCORE, Grade: $FK_GRADE, Words: $WORD_COUNT)${NC}"
            FAILED_FILES+=("$FILE_NAME: Score $FLESCH_SCORE not in range $MIN_SCORE-$MAX_SCORE")
        fi
    fi
done

# Summary
echo ""
echo -e "${YELLOW}================================${NC}"
echo -e "${YELLOW}Readability Summary${NC}"
echo -e "${YELLOW}================================${NC}"
echo ""
echo -e "${GREEN}Passed: ${#PASSED_FILES[@]}${NC}"
for file in "${PASSED_FILES[@]}"; do
    echo "  ✓ $file"
done
echo ""

if [ ${#SKIPPED_FILES[@]} -gt 0 ]; then
    echo -e "${YELLOW}Skipped: ${#SKIPPED_FILES[@]}${NC}"
    for file in "${SKIPPED_FILES[@]}"; do
        echo "  ⊘ $file"
    done
    echo ""
fi

if [ ${#FAILED_FILES[@]} -gt 0 ]; then
    echo -e "${RED}Failed: ${#FAILED_FILES[@]}${NC}"
    for file in "${FAILED_FILES[@]}"; do
        echo "  ✗ $file"
    done
    echo ""
    echo -e "${YELLOW}Tip: Flesch Reading Ease score 60-70 = Grade 9-12${NC}"
    echo "  - Simplify complex sentences"
    echo "  - Use shorter words when possible"
    echo "  - Break long paragraphs into shorter ones"
    echo "  - Define technical terms before using them"
    echo ""
    exit 1
else
    echo -e "${GREEN}All files meet readability requirements!${NC}"
    exit 0
fi
