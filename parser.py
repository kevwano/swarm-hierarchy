import re
import matplotlib.pyplot as plt

# ---------------------------------------------------------
# Helper function to parse a log file
# ---------------------------------------------------------
def parse_log(filename):
    times = []
    proportions = []

    # Track current opinions of all robots
    opinions = {}

    # Regex patterns
    start_dissemination = re.compile(
        r"robot (\d+) : start dissemination opinion: (white|black) , currenttime ([0-9.]+)"
    )

    with open(filename, "r") as f:
        for line in f:
            m = start_dissemination.search(line)
            if m:
                robot = int(m.group(1))
                opinion = m.group(2)
                t = float(m.group(3))

                # Update robot's opinion
                opinions[robot] = opinion

                # Compute proportion white
                if len(opinions) > 0:
                    white_count = sum(1 for v in opinions.values() if v == "white")
                    prop_white = white_count / len(opinions)

                    times.append(t)
                    proportions.append(prop_white)

    return times, proportions


# ---------------------------------------------------------
# Parse all three files
# ---------------------------------------------------------
files = [
   ("test31B-1-hierarchical.txt", "Run 1"),
    ("test31B-2-hierarchical.txt", "Run 2"),
   ("test31B-3-hierarchical.txt", "Run 3"),
]

plt.figure(figsize=(12, 7))

for filename, label in files:
    t, p = parse_log(filename)
    plt.plot(t, p, label=label)

# ---------------------------------------------------------
# Final plot formatting
# ---------------------------------------------------------
plt.xlabel("Time (seconds)")
plt.ylabel("Proportion of robots with opinion = white")
plt.title("Proportion of Robots with Opinion White Over Time - 31 Black Tiles - Hierarchical Model")
plt.ylim(0, 1)
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()
