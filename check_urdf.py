import os
import sys

# 에러 로그에 뜬 정확한 경로
target_path = "/home/jy/ros2_ws/src/GeoRT/assets/ASSY_Hand_R/test_fixed.urdf"

print("="*50)
print(f"진단 시작: {target_path}")
print("="*50)

# 1. 파일 존재 확인
if not os.path.exists(target_path):
    print("❌ [치명적 에러] 파이썬이 보기에 파일이 '없습니다'.")
    print("   -> 현재 폴더의 실제 파일 목록:")
    folder = os.path.dirname(target_path)
    if os.path.exists(folder):
        for f in os.listdir(folder):
            print(f"      - {f}")
    else:
        print(f"      - 폴더 경로 자체가 없습니다: {folder}")
    sys.exit()
else:
    print("✅ 파일이 존재합니다.")

# 2. 파일 크기 확인 (빈 파일인지)
size = os.path.getsize(target_path)
print(f"ℹ️ 파일 크기: {size} bytes")
if size < 10:
    print("❌ [치명적 에러] 파일이 비어있거나 너무 작습니다. 내용이 저장되지 않았습니다.")
    sys.exit()

# 3. 읽기 권한 확인
if not os.access(target_path, os.R_OK):
    print("❌ [권한 에러] 읽기 권한이 없습니다. 'chmod +r' 명령어가 필요합니다.")
    sys.exit()
else:
    print("✅ 읽기 권한이 있습니다.")

# 4. 파일 내용 앞부분 확인 (인코딩/헤더 문제 확인)
try:
    with open(target_path, 'r', encoding='utf-8') as f:
        head = f.read(100)
        print(f"✅ 파일 내용 읽기 성공. 첫 100글자:\n{'-'*20}\n{head}\n{'-'*20}")
        if "xml" not in head:
             print("⚠️ [경고] XML 헤더(<?xml...)가 보이지 않습니다. URDF 형식이 아닐 수 있습니다.")
except Exception as e:
    print(f"❌ [인코딩 에러] 파일을 텍스트로 읽을 수 없습니다: {e}")
    sys.exit()

# 5. SAPIEN으로 로딩 시도
print("\n🔄 SAPIEN 로더 테스트 중...")
try:
    import sapien.core as sapien
    engine = sapien.Engine()
    renderer = sapien.SapienRenderer()
    engine.set_renderer(renderer)
    loader = engine.create_scene().create_urdf_loader()
    
    # 로딩 시도
    loader.load(target_path)
    print("🎉 [성공] SAPIEN이 URDF를 정상적으로 불러왔습니다!")
    print("   -> 이제 원래 프로그램을 실행해도 됩니다.")
except Exception as e:
    print(f"❌ [SAPIEN 내부 에러] 파일은 있지만 시뮬레이터가 거부했습니다:\n{e}")

print("="*50)
