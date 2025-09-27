import re

def convert_to_xy_and_save(file_path, output_path):
    points = []
    with open(file_path, 'r') as file:
        for line in file:
            # 줄에서 숫자(정수, 소수 포함)를 모두 추출
            nums = re.findall(r"[-+]?\d*\.\d+|\d+", line)
            if len(nums) >= 2:  # 숫자가 2개 이상 있으면
                x = float(nums[0])
                y = float(nums[1])
                points.append((x, y))

    # (x, y) 형식으로 새 파일에 저장
    with open(output_path, 'w') as output_file:
        for point in points:
            output_file.write(f"({point[0]}, {point[1]})\n")

# === 여기서 바로 실행 ===
convert_to_xy_and_save("output1.txt", "convert1.txt")
print("Data has been saved to convert1.txt")

convert_to_xy_and_save("output2.txt", "convert2.txt")
print("Data has been saved to convert2.txt")

