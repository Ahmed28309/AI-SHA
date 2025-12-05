#!/usr/bin/env python3
import os
import sys
import time
import json
import uuid
import requests
import smtplib
import imaplib
import email
from email.message import EmailMessage
from typing import List

from rich.console import Console
from rich.table import Table
from rich import box
from rich.text import Text
from prompt_toolkit import PromptSession
from prompt_toolkit.completion import WordCompleter

console = Console()
session = PromptSession()

NOTIFIER_EMAIL = "lambdabotai@gmail.com"      
RECIPIENT_EMAILS = [                         
    "srinjaycode@gmail.com",
    "yousef.saleh456@gmail.com",
    "pixelsauras@gmail.com",
    "ahmed.rizsoft@gmail.com",
    "hamzahusseini2008@gmail.com",
    "jameelon8@gmail.com"
]

SMTP_SERVER = "smtp.gmail.com"
SMTP_PORT = 587
SMTP_USER = "lambdabotai@gmail.com"
SMTP_PASS = "weqdnfvluceqaitl"

IMAP_SERVER = "imap.gmail.com"
IMAP_USER = "lambdabotai@gmail.com"
IMAP_PASS = "weqdnfvluceqaitl"

API_BASE = os.environ.get("LAMBDA_CLOUD_API_BASE", "https://cloud.lambdalabs.com/api/v1")
API_KEY = os.environ.get("LAMBDA_CLOUD_API_KEY")

HEADERS = {
    "Authorization": f"Bearer {API_KEY}" if API_KEY else "",
    "Content-Type": "application/json",
}

def api_get(path):
    r = requests.get(API_BASE + path, headers=HEADERS)
    r.raise_for_status()
    return r.json().get("data")

def api_post(endpoint, payload):
    headers = {
        "Authorization": f"Bearer {API_KEY}",
        "Content-Type": "application/json"
    }
    r = requests.post(API_BASE + endpoint, headers=headers, json=payload)

    # Print API error cleanly if present
    if r.status_code != 200:
        try:
            print("API ERROR:", r.json())
        except Exception:
            print("API ERROR:", r.text)

    return r   

def list_instance_types():
    return api_get("/instance-types")


def list_instances():
    return api_get("/instances")

def pretty_gpus(gpu_data):
    table = Table(title="Lambda Cloud GPU Inventory", box=box.HEAVY)
    table.add_column("Index")
    table.add_column("Type", style="cyan")
    table.add_column("GPU", style="green")
    table.add_column("#")
    table.add_column("$/hr")
    table.add_column("Availability")
    table.add_column("Regions", style="yellow")

    keys = list(gpu_data.keys())

    for idx, key in enumerate(keys):
        it = gpu_data[key]["instance_type"]
        regions = gpu_data[key].get("regions_with_capacity_available", [])
        price = f"${it['price_cents_per_hour']/100:.2f}"
        available = "YES" if regions else "NO"
        color = "green" if regions else "red"
        region_str = ", ".join([r["description"] for r in regions]) if regions else "—"

        table.add_row(
            str(idx),
            it["name"],
            it["gpu_description"],
            str(it["specs"]["gpus"]),
            price,
            Text(available, style=color),
            region_str,
        )

    console.print(table)

def pretty_instances(instances):
    table = Table(title="Running Instances", box=box.ROUNDED)
    table.add_column("ID")
    table.add_column("GPU")
    table.add_column("Region")
    table.add_column("Status")
    table.add_column("IP")
    table.add_column("$/hr")

    for inst in instances:
        it = inst.get("instance_type", {})
        price = f"${it.get('price_cents_per_hour',0)/100:.2f}" if it else "?"
        status_color = "green" if inst.get("status") == "running" else "yellow"

        table.add_row(
            inst.get("id", "?"),
            it.get("name", "?"),
            inst.get("region", {}).get("description", "?"),
            Text(inst.get("status", "?"), style=status_color),
            inst.get("ip", "—"),
            price,
        )

    console.print(table)

def interactive_gpu_selector():
    gpus = list_instance_types()
    keys = list(gpus.keys())

    pretty_gpus(gpus)

    try:
        sel = int(input("Select GPU index to launch: "))
    except Exception:
        console.print("[red]Invalid input[/red]")
        return

    if sel < 0 or sel >= len(keys):
        console.print("[red]Index out of range[/red]")
        return

    chosen_block = gpus[keys[sel]]
    chosen_type = chosen_block["instance_type"]["name"]
    regions = chosen_block.get("regions_with_capacity_available", [])

    if not regions:
        console.print("[bold red]This GPU has NO AVAILABLE REGIONS. Cannot launch.[/bold red]")
        return

    chosen_region = regions[0]["name"]  

    launch_instance(chosen_type, chosen_region)

def launch_instance(instance_type, region):
    payload = {
        "instance_type_name": instance_type,
        "region_name": region,
        "ssh_key_names": ["isaac-key"],
        "image_name": "lambda-stack-22.04"
    }

    r = api_post("/instance-operations/launch", payload)

    if r.status_code != 200:
        print("API ERROR:", r.text)
        r.raise_for_status()

    console.print("[bold green]Instance launched successfully:[/bold green]")
    console.print(r.json())

def send_alert(subject, body):
    msg = EmailMessage()
    msg["From"] = NOTIFIER_EMAIL
    msg["To"] = ", ".join(RECIPIENT_EMAILS)
    msg["Subject"] = subject
    msg.set_content(body)

    with smtplib.SMTP(SMTP_SERVER, SMTP_PORT) as s:
        s.starttls()
        s.login(SMTP_USER, SMTP_PASS)
        s.send_message(msg)


def wait_for_reply(token):
    M = imaplib.IMAP4_SSL(IMAP_SERVER)
    M.login(IMAP_USER, IMAP_PASS)

    while True:
        M.select("INBOX")
        _, data = M.search(None, f'(BODY "{token}")')
        if data[0].split():
            return True
        time.sleep(15)

def watch_rtx():
    token = str(uuid.uuid4())[:8]
    models = ["RTX 6000", "A6000"]

    console.print("[bold cyan]Watching RTX6000 / A6000 availability...[/bold cyan]")

    while True:
        gpus = list_instance_types()

        for k, v in gpus.items():
            it = v["instance_type"]
            regions = v.get("regions_with_capacity_available", [])

            if any(m.lower() in it["gpu_description"].lower() for m in models) and regions:
                region = regions[0]["name"]

                subject = f"RTX/A6000 AVAILABLE [{token}]"
                body = f"GPU: {it['name']}\nRegion: {region}\nReply with this token to auto-launch: {token}"
                send_alert(subject, body)

                console.print("[bold green]Availability detected! Email sent.[/bold green]")

                if wait_for_reply(token):
                    console.print("[bold yellow]Reply detected. Launching now...[/bold yellow]")
                    launch_instance(it["name"], region)
                    return

        time.sleep(10)

def repl():
    completer = WordCompleter([
        "list-gpus",
        "list-instances",
        "launch",
        "watch-rtx",
        "exit"
    ], ignore_case=True)

    console.print("[bold cyan]Lambda Cloud Interactive Shell[/bold cyan]")
    console.print("Commands: list-gpus | list-instances | launch | watch-rtx | exit")

    while True:
        try:
            cmd = session.prompt("lambda> ", completer=completer).strip()

            if cmd == "list-gpus":
                pretty_gpus(list_instance_types())

            elif cmd == "list-instances":
                pretty_instances(list_instances())

            elif cmd == "launch":
                interactive_gpu_selector()

            elif cmd == "watch-rtx":
                watch_rtx()

            elif cmd == "exit":
                console.print("[red]Exiting...[/red]")
                return
        except KeyboardInterrupt:
            continue

if __name__ == "__main__":
    repl()
