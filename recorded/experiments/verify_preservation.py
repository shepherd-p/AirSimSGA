import json
from ag.storage import ROOT, verify_originals, write_json
if __name__ == '__main__':
    result = verify_originals()
    write_json(ROOT/'verification/preservation_latest.json',result)
    print(json.dumps(result,indent=2,ensure_ascii=False))
    raise SystemExit(0 if result['unchanged'] else 2)
