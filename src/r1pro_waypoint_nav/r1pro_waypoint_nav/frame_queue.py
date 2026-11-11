"""Bounded latest-ready scheduling; never substitute latest TF for frame-time TF."""


def take_latest_ready(pending, prepare, fresh, now, retry_errors, timeout=.25):
    items=list(pending);discard=set();selected=None;superseded=0;error=''
    for index in range(len(items)-1,-1,-1):
        msg,received=items[index]
        if not fresh(msg):
            discard.add(index);error='stale_or_invalid_timestamp';continue
        try:
            prepared=prepare(msg)
        except retry_errors as exc:
            error=str(exc)
            if now-received>timeout:discard.add(index)
            continue
        selected=(msg,received,prepared)
        superseded=index
        discard.update(range(index+1))
        break
    pending.clear()
    pending.extend(item for i,item in enumerate(items) if i not in discard)
    return selected,len(discard)-int(selected is not None),superseded,error
